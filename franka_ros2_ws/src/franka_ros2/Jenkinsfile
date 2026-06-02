#!groovy

/**
 * Clone a repository from either a private URL (if provided) or from dependency.repos config.
 *
 * @param repoName       Target directory name for the clone
 * @param privateUrl     Private repo URL (empty string to use public)
 * @param privateBranch  Branch to use for private repo
 * @param publicUrl      Public URL from dependency.repos
 * @param publicVersion  Version/branch from dependency.repos
 * @param credentialId   Jenkins SSH credential ID
 * @param recursive      Whether to clone recursively (default: false)
 * @param fetchTags      Whether to fetch tags after clone (default: false)
 */
def cloneRepo(Map args) {
  def repoName = args.repoName
  def privateUrl = args.privateUrl ?: ''
  def privateBranch = args.privateBranch ?: 'main'
  def publicUrl = args.publicUrl
  def publicVersion = args.publicVersion ?: 'main'
  def credentialId = args.credentialId
  def recursive = args.recursive ?: false
  def fetchTags = args.fetchTags ?: false

  def recursiveFlag = recursive ? '--recursive' : ''
  def fetchTagsCmd = fetchTags ? "&& cd ${repoName} && git fetch --tags" : ''

  if (privateUrl) {
    echo "Cloning ${repoName} from internal: ${privateUrl} (branch: ${privateBranch})"
    sshagent(credentials: [credentialId]) {
      sh """
        git clone ${recursiveFlag} --branch ${privateBranch} --single-branch ${privateUrl} ${repoName} ${fetchTagsCmd}
      """
    }
  } else {
    echo "Cloning ${repoName} from dependency.repos: ${publicUrl} (version: ${publicVersion})"
    sh """
      git clone ${recursiveFlag} --branch ${publicVersion} --single-branch ${publicUrl} ${repoName} ${fetchTagsCmd}
    """
  }
}

pipeline {
  agent {
    label 'docker'
  }

  triggers {
    pollSCM('H/5 * * * *')
  }

  parameters {
    string(name: 'libfrankaRepoUrl',
           defaultValue: 'ssh://git@bitbucket.fe.lan:7999/moctrl/libfranka.git',
           description: 'SSH URL to clone libfranka from internal repo. Leave empty to use the github remote.')

    string(name: 'libfrankaBranch',
           defaultValue: 'main',
           description: 'Branch or tag to checkout for libfranka.')

    string(name: 'frankaDescriptionRepoUrl',
           defaultValue: 'ssh://git@bitbucket.fe.lan:7999/moctrl/franka_description.git',
           description: 'SSH URL to clone franka_description from internal repo. Leave empty to use the github remote.')

    string(name: 'frankaDescriptionBranch',
           defaultValue: 'jazzy',
           description: 'Branch or tag to checkout for franka_description.')

    string(name: 'sshCredentialId',
           defaultValue: 'git_ssh',
           description: 'Jenkins credential ID for SSH key to access internal Bitbucket repos.')
  }

  stages {
    stage('Get Ready') {
      steps {
        cleanWs()
        checkout scm
        script {
          notifyBitbucket()
          currentBuild.displayName = "[libfranka: ${params.libfrankaRepoUrl ? params.libfrankaBranch : 'private'}, franka_description: ${params.frankaDescriptionRepoUrl ? params.frankaDescriptionBranch : 'private'}]"
        }
        sh 'rm -rf build log install libfranka franka_description'
      }
    }

    stage('Fetch Dependencies') {
      steps {
        script {
          sh 'rm -rf libfranka franka_description'

          // Read defaults from dependencies.repos so we can follow the versions defined there
          def repos = readYaml file: 'dependency.repos'
          def repoMap = repos?.repositories ?: [:]

          // Helper to get repo config with defaults
          def getRepoConfig = { name, defaultUrl ->
            def cfg = repoMap[name] ?: [:]
            return [
              url: cfg.url ?: defaultUrl,
              version: cfg.version ?: 'main'
            ]
          }

          def libfrankaCfg = getRepoConfig('libfranka', 'https://github.com/frankarobotics/libfranka.git')
          def frankaDescCfg = getRepoConfig('franka_description', 'https://github.com/frankarobotics/franka_description.git')

          // Clone libfranka
          cloneRepo(
            repoName: 'libfranka',
            privateUrl: params.libfrankaRepoUrl,
            privateBranch: params.libfrankaBranch,
            publicUrl: libfrankaCfg.url,
            publicVersion: libfrankaCfg.version,
            credentialId: params.sshCredentialId,
            recursive: true,
            fetchTags: true
          )

          // Clone franka_description
          cloneRepo(
            repoName: 'franka_description',
            privateUrl: params.frankaDescriptionRepoUrl,
            privateBranch: params.frankaDescriptionBranch,
            publicUrl: frankaDescCfg.url,
            publicVersion: frankaDescCfg.version,
            credentialId: params.sshCredentialId
          )

          // Stash cloned external dependencies so later stages (running in another agent/container)
          // can restore them even if the workspace is wiped or a different node is used.
          // useDefaultExcludes: false is required to include .git directories for version detection
          stash name: 'external_deps', includes: 'libfranka/**,franka_description/**', useDefaultExcludes: false

          sh 'echo "=== Workspace structure ===" && ls -la'
        }
      }
    }

    stage('Build') {
      agent {
        dockerfile {
          reuseNode true
        }
      }
      steps {
        // Clean any existing files before unstashing to avoid permission conflicts
        sh 'rm -rf libfranka franka_description'
        // Restore external deps cloned in the Fetch Dependencies stage
        unstash 'external_deps'
        sh '''
          . /opt/ros/$ROS_DISTRO/setup.sh
          echo "=== Workspace structure ===" && ls -la
          colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=ON -DBUILD_TESTS=OFF
        '''
      }
    }

    stage('Test') {
      agent {
        dockerfile {
          reuseNode true
        }
      }
      steps {
        sh '''
          . /opt/ros/$ROS_DISTRO/setup.sh
          . install/setup.sh
          colcon test --packages-ignore hardware_interface realtime_tools libfranka controller_manager integration_launch_testing --event-handlers console_direct+
          colcon test-result --verbose
        '''
      }
      post {
        always {
          junit 'build/**/test_results/**/*.xml'
        }
      }
    }
  }
  post {
      always {
          cleanWs()
          script {
              notifyBitbucket()
          }
      }
  }
}

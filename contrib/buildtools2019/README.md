# How to set-up a Windows Docker for CI

This instruction is based on:

* https://docs.microsoft.com/en-us/visualstudio/install/build-tools-container?view=vs-2019
* https://docs.microsoft.com/en-us/visualstudio/install/advanced-build-tools-container?view=vs-2019
* https://github.com/git-for-windows/git/wiki/Silent-or-Unattended-Installation
* https://docs.gitlab.com/runner/install/windows.html
* https://docs.gitlab.com/runner/executors/docker.html

## Building the Docker image

1. Install Docker and set it to Windows mode
1. In a single directory, place:
   * This [Dockerfile](/contrib/buildtools2019/Dockerfile)
   * This [Install.cmd](/contrib/buildtools2019/Install.cmd)
   * [A specific version of the Git for Windows installer](https://github.com/git-for-windows/git/releases/download/v2.29.1.windows.1/Git-2.29.1-64-bit.exe) from a [specific release](https://github.com/git-for-windows/git/releases/tag/v2.29.1.windows.1) named `Git-2.29.1-64-bit.exe` (exactly the same as in the Dockerfile)
1. Run: `docker build -t buildtools2019:latest -m 2GB .`

## Register the GitLab runner

1. Install the GitLab runner, see https://docs.gitlab.com/runner/install/windows.html
1. Register the GitLab runner with `gitlab-runner.exe register` (ask the software coordinators for the credentials)
1. Add a runner configuration, set `pull_policy` to `"never"` (see https://docs.gitlab.com/runner/executors/docker.html), for example:
```
   [runners.docker]
    tls_verify = false
    image = "buildtools2019"
    privileged = false
    disable_entrypoint_overwrite = false
    oom_kill_disable = false
    disable_cache = false
    volumes = ["c:\\cache"]
    pull_policy = "never"
    shm_size = 0

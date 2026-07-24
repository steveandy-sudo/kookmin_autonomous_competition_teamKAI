[CmdletBinding()]
param(
    [string]$Distro = "Ubuntu-22.04",
    [switch]$Headless,
    [switch]$NoCoordinateGui,
    [switch]$EnableRviz,
    [switch]$NoBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$wslRepoRoot = (
    wsl.exe -d $Distro -- wslpath -a $repoRoot
).Trim()

if (-not $wslRepoRoot) {
    throw "작업 폴더를 WSL 경로로 변환하지 못했습니다: $repoRoot"
}

$escapedRoot = $wslRepoRoot.Replace("'", "'\''")
$headlessValue = if ($Headless) { "true" } else { "false" }
$guiValue = if ($NoCoordinateGui) { "false" } else { "true" }
$rvizValue = if ($EnableRviz) { "true" } else { "false" }

$commands = @(
    "set -e",
    "source /opt/ros/humble/setup.bash",
    "cd '$escapedRoot/xycar_ws'"
)

if (-not $NoBuild) {
    $commands += "colcon build --packages-up-to xycar_gazebo_bridge --symlink-install"
}

$commands += @(
    "source install/setup.bash",
    "ros2 launch xycar_gazebo_bridge slam_map_gazebo.launch.py headless:=$headlessValue start_coordinate_gui:=$guiValue enable_rviz:=$rvizValue"
)

Write-Host "Team KAI SLAM Gazebo를 $Distro 에서 시작합니다."
wsl.exe -d $Distro -- bash -lc ($commands -join " && ")

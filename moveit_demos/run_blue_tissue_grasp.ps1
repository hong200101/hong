# ============================================================================
# 蓝色纸巾包视觉抓取系统启动脚本 (PowerShell 版本)
# 用法: .\run_blue_tissue_grasp.ps1 [选项]
# ============================================================================

param(
    [switch]$Help,
    [string]$Config = "",
    [switch]$Auto,
    [switch]$Place,
    [switch]$NoViz
)

# 颜色输出函数
function Write-ColorOutput($ForegroundColor) {
    $fc = $host.UI.RawUI.ForegroundColor
    $host.UI.RawUI.ForegroundColor = $ForegroundColor
    if ($args) {
        Write-Output $args
    }
    $host.UI.RawUI.ForegroundColor = $fc
}

# 打印标题
function Print-Header {
    Write-ColorOutput Cyan "======================================================================"
    Write-ColorOutput Cyan "           蓝色纸巾包视觉抓取系统启动脚本"
    Write-ColorOutput Cyan "======================================================================"
}

# 打印信息
function Print-Info($Message) {
    Write-ColorOutput Green "[信息] $Message"
}

# 打印警告
function Print-Warning($Message) {
    Write-ColorOutput Yellow "[警告] $Message"
}

# 打印错误
function Print-Error($Message) {
    Write-ColorOutput Red "[错误] $Message"
}

# 显示帮助
function Show-Help {
    Write-Output "用法: .\run_blue_tissue_grasp.ps1 [选项]"
    Write-Output ""
    Write-Output "选项:"
    Write-Output "  -Help               显示此帮助信息"
    Write-Output "  -Config <file>      指定配置文件"
    Write-Output "  -Auto               自动运行模式（非交互）"
    Write-Output "  -Place              抓取后放置物体"
    Write-Output "  -NoViz              不显示可视化"
    Write-Output ""
    Write-Output "示例:"
    Write-Output "  .\run_blue_tissue_grasp.ps1                # 交互式模式"
    Write-Output "  .\run_blue_tissue_grasp.ps1 -Auto          # 自动运行一次抓取"
    Write-Output "  .\run_blue_tissue_grasp.ps1 -Auto -Place   # 自动运行抓取并放置"
    Write-Output ""
}

# 检查依赖
function Check-Dependencies {
    Print-Info "检查依赖..."
    
    $modules = @("pyrealsense2", "cv2", "numpy", "yaml")
    $allInstalled = $true
    
    foreach ($module in $modules) {
        try {
            python -c "import $module" 2>$null
            if ($LASTEXITCODE -ne 0) {
                Print-Error "未安装 $module"
                $allInstalled = $false
            }
        } catch {
            Print-Error "未安装 $module"
            $allInstalled = $false
        }
    }
    
    if ($allInstalled) {
        Print-Info "✓ 所有依赖已安装"
        return $true
    } else {
        Print-Error "依赖检查失败"
        Print-Info "安装命令: pip3 install pyrealsense2 opencv-python numpy pyyaml"
        return $false
    }
}

# 主函数
function Main {
    Print-Header
    
    # 显示帮助
    if ($Help) {
        Show-Help
        exit 0
    }
    
    # 检查依赖
    if (-not (Check-Dependencies)) {
        Print-Error "请先安装缺失的依赖"
        exit 1
    }
    
    # 构建参数
    $args = @()
    
    if ($Config) {
        $args += "--config"
        $args += $Config
    }
    
    if ($Auto) {
        $args += "--auto"
    }
    
    if ($Place) {
        $args += "--place"
    }
    
    if (-not $NoViz) {
        $args += "--visualize"
    }
    
    # 显示运行模式
    Write-Output ""
    if ($Auto) {
        Print-Info "运行模式: 自动"
        if ($Place) {
            Print-Info "  - 抓取后放置物体"
        }
    } else {
        Print-Info "运行模式: 交互式"
    }
    
    # 检查机械臂控制节点
    Write-Output ""
    Print-Warning "请确保已启动机械臂控制节点!"
    Write-Output ""
    Write-Output "  终端1: 启动机械臂控制（带 MoveIt）"
    Write-Output "  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \"
    Write-Output "    can_port:=can0 arm_type:=piper effector_type:=agx_gripper"
    Write-Output ""
    
    $response = Read-Host "机械臂控制节点已启动? (y/n)"
    if ($response -notmatch '^[Yy]$') {
        Print-Error "请先启动机械臂控制节点"
        exit 1
    }
    
    # 启动视觉抓取系统
    Print-Info "启动视觉抓取系统..."
    Write-Output ""
    
    # 获取脚本目录
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    Set-Location $scriptDir
    
    # 运行 Python 程序
    python blue_tissue_grasp.py @args
    
    # 检查退出状态
    if ($LASTEXITCODE -eq 0) {
        Write-Output ""
        Print-Info "✓ 系统正常退出"
    } else {
        Write-Output ""
        Print-Error "✗ 系统异常退出"
    }
}

# 运行主函数
Main

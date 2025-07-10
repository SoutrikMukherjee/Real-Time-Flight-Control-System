#!/bin/bash
#
# RTFCS Deployment Script
# Deploy flight controller software to target hardware
#
# Usage: ./deploy.sh --target <IP> [options]
#

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
TARGET_IP=""
TARGET_USER="root"
TARGET_PATH="/opt/rtfcs"
BUILD_DIR="build"
CONFIG_FILE=""
VERIFY=false
BACKUP=true
RESTART=true
PORT=22

# Print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Display usage
usage() {
    cat << EOF
Usage: $0 --target <IP> [options]

Deploy RTFCS flight controller to target hardware

Options:
    -t, --target <IP>       Target device IP address (required)
    -u, --user <USER>       SSH user (default: root)
    -p, --port <PORT>       SSH port (default: 22)
    -P, --path <PATH>       Target installation path (default: /opt/rtfcs)
    -c, --config <FILE>     Configuration file to deploy
    -b, --build-dir <DIR>   Build directory (default: build)
    -v, --verify            Verify deployment after transfer
    -n, --no-backup         Don't backup existing installation
    -s, --no-restart        Don't restart service after deployment
    -h, --help              Show this help message

Examples:
    $0 --target 192.168.1.100
    $0 --target 192.168.1.100 --config config/quadcopter.yaml --verify
    $0 -t 10.0.0.50 -u pi -p 2222 --no-restart
EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -t|--target)
                TARGET_IP="$2"
                shift 2
                ;;
            -u|--user)
                TARGET_USER="$2"
                shift 2
                ;;
            -p|--port)
                PORT="$2"
                shift 2
                ;;
            -P|--path)
                TARGET_PATH="$2"
                shift 2
                ;;
            -c|--config)
                CONFIG_FILE="$2"
                shift 2
                ;;
            -b|--build-dir)
                BUILD_DIR="$2"
                shift 2
                ;;
            -v|--verify)
                VERIFY=true
                shift
                ;;
            -n|--no-backup)
                BACKUP=false
                shift
                ;;
            -s|--no-restart)
                RESTART=false
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done
}

# Validate arguments
validate_args() {
    if [[ -z "$TARGET_IP" ]]; then
        print_error "Target IP address is required"
        usage
        exit 1
    fi
    
    if [[ ! -d "$BUILD_DIR" ]]; then
        print_error "Build directory not found: $BUILD_DIR"
        exit 1
    fi
    
    if [[ ! -f "$BUILD_DIR/rtfcs_flight_controller" ]]; then
        print_error "Flight controller executable not found in $BUILD_DIR"
        print_info "Please build the project first: mkdir build && cd build && cmake .. && make"
        exit 1
    fi
    
    if [[ -n "$CONFIG_FILE" && ! -f "$CONFIG_FILE" ]]; then
        print_error "Configuration file not found: $CONFIG_FILE"
        exit 1
    fi
}

# Check SSH connection
check_connection() {
    print_info "Checking connection to $TARGET_IP:$PORT..."
    
    if ! ssh -q -o ConnectTimeout=5 -p "$PORT" "$TARGET_USER@$TARGET_IP" exit; then
        print_error "Failed to connect to $TARGET_IP:$PORT"
        exit 1
    fi
    
    print_info "Connection successful"
}

# Get target system info
get_target_info() {
    print_info "Getting target system information..."
    
    local arch=$(ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "uname -m")
    local kernel=$(ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "uname -r")
    local mem=$(ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "free -h | grep Mem | awk '{print \$2}'")
    
    print_info "Target architecture: $arch"
    print_info "Target kernel: $kernel"
    print_info "Target memory: $mem"
    
    # Verify architecture compatibility
    if [[ "$arch" != "aarch64" && "$arch" != "armv7l" ]]; then
        print_warn "Target architecture may not be compatible: $arch"
    fi
}

# Backup existing installation
backup_existing() {
    if [[ "$BACKUP" == false ]]; then
        return
    fi
    
    print_info "Backing up existing installation..."
    
    local backup_dir="$TARGET_PATH.backup.$(date +%Y%m%d_%H%M%S)"
    
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "
        if [[ -d '$TARGET_PATH' ]]; then
            mkdir -p '$backup_dir'
            cp -r '$TARGET_PATH'/* '$backup_dir/' || true
            echo 'Backup created at: $backup_dir'
        else
            echo 'No existing installation found'
        fi
    "
}

# Stop the service
stop_service() {
    print_info "Stopping RTFCS service..."
    
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "
        if systemctl is-active --quiet rtfcs; then
            systemctl stop rtfcs
            echo 'Service stopped'
        else
            echo 'Service not running'
        fi
    "
}

# Deploy files
deploy_files() {
    print_info "Deploying files to $TARGET_IP:$TARGET_PATH..."
    
    # Create target directory
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "mkdir -p '$TARGET_PATH/bin' '$TARGET_PATH/config' '$TARGET_PATH/scripts'"
    
    # Copy executable
    print_info "Copying executable..."
    scp -P "$PORT" "$BUILD_DIR/rtfcs_flight_controller" "$TARGET_USER@$TARGET_IP:$TARGET_PATH/bin/"
    
    # Copy libraries if any
    if [[ -d "$BUILD_DIR/lib" ]]; then
        print_info "Copying libraries..."
        scp -P "$PORT" -r "$BUILD_DIR/lib" "$TARGET_USER@$TARGET_IP:$TARGET_PATH/"
    fi
    
    # Copy configuration
    if [[ -n "$CONFIG_FILE" ]]; then
        print_info "Copying configuration file..."
        scp -P "$PORT" "$CONFIG_FILE" "$TARGET_USER@$TARGET_IP:$TARGET_PATH/config/config.yaml"
    else
        # Copy default configs
        print_info "Copying default configuration files..."
        scp -P "$PORT" config/*.yaml "$TARGET_USER@$TARGET_IP:$TARGET_PATH/config/" || true
    fi
    
    # Copy scripts
    print_info "Copying utility scripts..."
    scp -P "$PORT" scripts/*.py scripts/*.sh "$TARGET_USER@$TARGET_IP:$TARGET_PATH/scripts/" || true
    
    # Set permissions
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "
        chmod +x '$TARGET_PATH/bin/rtfcs_flight_controller'
        chmod +x '$TARGET_PATH/scripts/'*.sh || true
    "
}

# Install systemd service
install_service() {
    print_info "Installing systemd service..."
    
    local service_file="/etc/systemd/system/rtfcs.service"
    
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "cat > $service_file" << EOF
[Unit]
Description=Real-Time Flight Control System
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=$TARGET_PATH
ExecStart=$TARGET_PATH/bin/rtfcs_flight_controller --config $TARGET_PATH/config/config.yaml --daemon
ExecStop=/bin/kill -TERM \$MAINPID
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

# Real-time settings
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=90
LimitRTPRIO=99
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
EOF
    
    # Reload systemd
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "systemctl daemon-reload"
    
    # Enable service
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "systemctl enable rtfcs"
}

# Verify deployment
verify_deployment() {
    if [[ "$VERIFY" == false ]]; then
        return
    fi
    
    print_info "Verifying deployment..."
    
    # Check executable
    if ! ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "test -x '$TARGET_PATH/bin/rtfcs_flight_controller'"; then
        print_error "Executable not found or not executable"
        exit 1
    fi
    
    # Test executable
    print_info "Testing executable..."
    local version=$(ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "'$TARGET_PATH/bin/rtfcs_flight_controller' --version" 2>&1)
    print_info "Deployed version: $version"
    
    # Check configuration
    if ! ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "test -f '$TARGET_PATH/config/config.yaml'"; then
        print_warn "Configuration file not found"
    fi
    
    print_info "Deployment verified successfully"
}

# Start the service
start_service() {
    if [[ "$RESTART" == false ]]; then
        print_info "Skipping service restart (--no-restart specified)"
        return
    fi
    
    print_info "Starting RTFCS service..."
    
    ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "systemctl start rtfcs"
    
    # Wait for service to start
    sleep 2
    
    # Check service status
    if ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "systemctl is-active --quiet rtfcs"; then
        print_info "Service started successfully"
        
        # Show recent logs
        print_info "Recent service logs:"
        ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "journalctl -u rtfcs -n 10 --no-pager"
    else
        print_error "Service failed to start"
        print_info "Service logs:"
        ssh -p "$PORT" "$TARGET_USER@$TARGET_IP" "journalctl -u rtfcs -n 20 --no-pager"
        exit 1
    fi
}

# Main deployment process
main() {
    print_info "RTFCS Deployment Script"
    print_info "======================"
    
    parse_args "$@"
    validate_args
    
    print_info "Deployment configuration:"
    print_info "  Target: $TARGET_USER@$TARGET_IP:$PORT"
    print_info "  Path: $TARGET_PATH"
    print_info "  Build directory: $BUILD_DIR"
    [[ -n "$CONFIG_FILE" ]] && print_info "  Config file: $CONFIG_FILE"
    
    # Confirm deployment
    read -p "Continue with deployment? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Deployment cancelled"
        exit 0
    fi
    
    # Deployment steps
    check_connection
    get_target_info
    stop_service
    backup_existing
    deploy_files
    install_service
    verify_deployment
    start_service
    
    print_info "Deployment completed successfully!"
    print_info "Access the target system: ssh -p $PORT $TARGET_USER@$TARGET_IP"
    print_info "View logs: ssh -p $PORT $TARGET_USER@$TARGET_IP 'journalctl -u rtfcs -f'"
}

# Run main function
main "$@"
# If you come from bash you might have to change your $PATH.
# export PATH=$HOME/bin:$HOME/.local/bin:/usr/local/bin:$PATH

# Path to your Oh My Zsh installation.
export ZSH="$HOME/.oh-my-zsh"

# Set name of the theme to load --- if set to "random", it will
# load a random theme each time Oh My Zsh is loaded, in which case,
# to know which specific one was loaded, run: echo $RANDOM_THEME
# See https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
ZSH_THEME="robbyrussell"

# Set list of themes to pick from when loading at random
# Setting this variable when ZSH_THEME=random will cause zsh to load
# a theme from this variable instead of looking in $ZSH/themes/
# If set to an empty array, this variable will have no effect.
# ZSH_THEME_RANDOM_CANDIDATES=( "robbyrussell" "agnoster" )

# Uncomment the following line to use case-sensitive completion.
# CASE_SENSITIVE="true"

# Uncomment the following line to use hyphen-insensitive completion.
# Case-sensitive completion must be off. _ and - will be interchangeable.
# HYPHEN_INSENSITIVE="true"

# Uncomment one of the following lines to change the auto-update behavior
# zstyle ':omz:update' mode disabled  # disable automatic updates
# zstyle ':omz:update' mode auto      # update automatically without asking
# zstyle ':omz:update' mode reminder  # just remind me to update when it's time

# Uncomment the following line to change how often to auto-update (in days).
# zstyle ':omz:update' frequency 13

# Uncomment the following line if pasting URLs and other text is messed up.
# DISABLE_MAGIC_FUNCTIONS="true"

# Uncomment the following line to disable colors in ls.
# DISABLE_LS_COLORS="true"

# Uncomment the following line to disable auto-setting terminal title.
# DISABLE_AUTO_TITLE="true"

# Uncomment the following line to enable command auto-correction.
# ENABLE_CORRECTION="true"

# Uncomment the following line to display red dots whilst waiting for completion.
# You can also set it to another string to have that shown instead of the default red dots.
# e.g. COMPLETION_WAITING_DOTS="%F{yellow}waiting...%f"
# Caution: this setting can cause issues with multiline prompts in zsh < 5.7.1 (see #5765)
# COMPLETION_WAITING_DOTS="true"

# Uncomment the following line if you want to disable marking untracked files
# under VCS as dirty. This makes repository status check for large repositories
# much, much faster.
# DISABLE_UNTRACKED_FILES_DIRTY="true"

# Uncomment the following line if you want to change the command execution time
# stamp shown in the history command output.
# You can set one of the optional three formats:
# "mm/dd/yyyy"|"dd.mm.yyyy"|"yyyy-mm-dd"
# or set a custom format using the strftime function format specifications,
# see 'man strftime' for details.
# HIST_STAMPS="mm/dd/yyyy"

# Would you like to use another custom folder than $ZSH/custom?
# ZSH_CUSTOM=/path/to/new-custom-folder

# Which plugins would you like to load?
# Standard plugins can be found in $ZSH/plugins/
# Custom plugins may be added to $ZSH_CUSTOM/plugins/
# Example format: plugins=(rails git textmate ruby lighthouse)
# Add wisely, as too many plugins slow down shell startup.
plugins=(git)

source $ZSH/oh-my-zsh.sh

# User configuration

# export MANPATH="/usr/local/man:$MANPATH"

# You may need to manually set your language environment
# export LANG=en_US.UTF-8

# Preferred editor for local and remote sessions
# if [[ -n $SSH_CONNECTION ]]; then
#   export EDITOR='vim'
# else
#   export EDITOR='nvim'
# fi

# Compilation flags
# export ARCHFLAGS="-arch $(uname -m)"

# Set personal aliases, overriding those provided by Oh My Zsh libs,
# plugins, and themes. Aliases can be placed here, though Oh My Zsh
# users are encouraged to define aliases within a top-level file in
# the $ZSH_CUSTOM folder, with .zsh extension. Examples:
# - $ZSH_CUSTOM/aliases.zsh
# - $ZSH_CUSTOM/macos.zsh
# For a full list of active aliases, run `alias`.
#
# Example aliases
# alias zshconfig="mate ~/.zshrc"
# alias ohmyzsh="mate ~/.oh-my-zsh"
migrate_to_ros2() {
    if [ -z "$1" ]; then
        echo "Usage: migrate_to_ros2 <original_bag.bag>"
        return 1
    fi

    local orig_bag="$1"
    local output_dir="${orig_bag%.bag}_ros2"


    echo "🔄 1/3: Converting bytes to ROS 2 CDR format..."
    # FIX: Added the missing --src flag right here!
    rosbags-convert --src "$orig_bag" --dst "$output_dir"

    if [ ! -d "$output_dir" ]; then
        echo "❌ Error: rosbags-convert failed."
        return 1
    fi

    echo "🔧 3/3: Patching the yaml-cpp bug in metadata.yaml..."
    # Python script to safely wipe all QoS profiles to empty strings
python3 -c "
import sys
filepath = '$output_dir/metadata.yaml'
try:
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    out_lines = []
    skip_mode = False
    base_indent = 0
    
    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(' '))
        
        # If we are skipping the giant QoS list
        if skip_mode:
            # Keep skipping if the line is deeper than the base indent, 
            # or if it's a list item starting with '-' at the same indent
            if stripped == '' or indent > base_indent or line.lstrip().startswith('-'):
                continue
            else:
                skip_mode = False # We hit the next real key (like serialization_format)

        # Look for the trigger word
        if 'offered_qos_profiles:' in line:
            # Keep the original indentation, but set it to an empty string
            out_lines.append(line[:indent] + 'offered_qos_profiles: \"\"\n')
            base_indent = indent
            
            # If it's just 'offered_qos_profiles:\n', trigger skip mode for the next lines
            if stripped.endswith(':'):
                skip_mode = True
            continue

        out_lines.append(line)
        
    with open(filepath, 'w') as f:
        f.writelines(out_lines)
except Exception as e:
    print(f'Warning: Could not patch YAML automatically: {e}')
"

  echo "✅ Done! Native ROS 2 bag is ready at: $output_dir"
}

play_galo_bag() {
  local DEFAULT_RATE=1.0
  local RATE
  local bags=()

  # Show usage if no arguments
  if [[ $# -eq 0 ]]; then
    echo "Usage: play_galo bag1 [bag2 ...] [rate]"
    return 1
  fi

  # Check if the last argument is a number (rate)
  if [[ $argv[-1] =~ '^[0-9]+(\.[0-9]+)?$' ]]; then
    RATE=$argv[-1]
    bags=(${argv[1,-2]})   # all arguments except the last
  else
    RATE=$DEFAULT_RATE
    bags=($argv)           # all arguments are bag files
  fi

  # Play each bag sequentially
  for bag in $bags; do
    echo "Playing $bag at rate $RATE"
    ros2 bag play "$bag" --clock --topics \
      /tf /Sensor/imu_front/data \
      /Sensor/lidar_front/rslidar_points \
      /Sensor/gnss/trimble_nmea_gga \
      /Sensor/gnss/orientation \
      /SC/state /SC/pure_state \
      /FB/wangle_feedback \
      /FB/wheel_speed_feedback \
      -r "$RATE"
  done
}

# # Play two bags with default rate (1.0)
# play_galo bag1.db3 bag2.db3

# # Play three bags with rate 0.5 (last argument is the rate)
# play_galo bag1.db3 bag2.db3 bag3.db3 0.5

# # Play a single bag with custom rate
# play_galo mybag.db3 2.0

autoload -U compinit && compinit
source /opt/ros/${ROS_DISTRO}/setup.zsh
[ -f ${WS}/install/setup.zsh ] && source ${WS}/install/setup.zsh
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_COLORIZED_OUTPUT=1
export TERM=xterm-256color

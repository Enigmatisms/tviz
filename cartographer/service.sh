default_output_path = "/home/llk/slam/trajectories/map1/"
if [ a"$1" = a -o a"$2" = a ]; then
    echo "Usage: ./service.sh
        <directory path to save your .pbstream file, ending with '/'>
        <name of .pbstream file>
        <trajectory file output path, ending with '/' > (optional)
    "
else
    if [ a"$3" != a ]; then
        default_output_path=$3
    else
        echo "Output path for trajectory txt file is not set."
        echo "Outputing to '$default_output_path' by default."
    fi
    rosservice call /finish_trajectory "trajectory_id: 0"
    rosservice call /write_state "$1$2.pbstream" true
    stream_convert "$1$2.pbstream" $default_output_path
fi

nums=(1 2 3 4 5)
suffix=("_0.25h" "_0.5" "_0.1")
for i in ${nums[@]}; do
    for ((j=0;j<3;j++)); do
        dir_name="hfps${i}_240${suffix[j]}"
        if [ ! -d $dir_name ]; then
            mkdir $dir_name
        else
            echo "'$dir_name' already exists."
        fi
    done
done
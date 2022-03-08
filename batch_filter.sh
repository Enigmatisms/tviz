a="./bag2.bag"

for file in `find . -ctime -0.05 -type f`; do
    suffix=${file##*.}
    if [ $suffix = "sh" ]; then
        continue
    fi
    no_prefix=${file#*/}
    pure_name=${no_prefix%.*}
    echo "Processing bag named '$pure_name'"
    python2 tf_filter.py $pure_name
    rm $file
    mv "./${pure_name}_st.bag" "./${pure_name}.bag"
done
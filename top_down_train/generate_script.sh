#!/bin/bash

in_dir_name=$1
out_file_name=$2
ft_dir_name=$3

file_names=`ls $in_dir_name/*.ply`

command="./build/box_decomposition"
scale=1

exec 1>$out_file_name

for file in $file_names
do
	
	command=$command" "$file" "$ft_dir_name" "$scale
	$command
	command="./build/box_decomposition"

done


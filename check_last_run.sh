now=$(date +"%s")

run_env_time=$(aws s3api list-objects --bucket dfwdr-blackbird71-training-stack-1-bucket-bj6bdmt50m44 --query 'Contents[?Key==`custom_files/run.env`].LastModified | [0]' | tr -d '"')
echo "Last run.env creation time is: $run_env_time"
run_env_time_epoch=$(date --date $run_env_time +%s)
time_diff=$((now-run_env_time_epoch))
echo "Time from run.env creation time is: $time_diff"

if [[ $time_diff -gt 900 ]]
then
	echo "Last run is more than 15min ago, can initiate new training now :D"
else
	wait_time=$((900-time_diff))
	echo "Last run is less than 15min, wait $wait_time seconds before initate next training..." 
fi

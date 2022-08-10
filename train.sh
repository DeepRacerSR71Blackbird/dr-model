pretrained_prefix=$(head -1 VERSION)
new_prefix=$(tail -1 VERSION)
time=$1
# EXAMPLE:
# new_prefix=lw2_on1_on5_vz1
# pretrained_prefix=lw2_on1_on5
# time=70

id="${new_prefix//_/-}"
echo "id to be plugged in new training job:"
echo $id

cp run.env.orig run.env
echo "">>run.env
echo "DR_LOCAL_S3_MODEL_PREFIX=$new_prefix">>run.env
echo "DR_LOCAL_S3_PRETRAINED_PREFIX=$pretrained_prefix">>run.env
echo "DR_LOCAL_S3_PRETRAINED_CHECKPOINT=best">>run.env

echo "============run.env file content============"
cat run.env
echo "============run.env file content============"

cd ..
echo "current directory:"
pwd

# ./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} $time
echo "./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} $time"
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} $time
#  &> logs/train-${id}.log &
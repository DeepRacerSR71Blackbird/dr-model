pretrained_prefix=$(head -1 VERSION)
new_prefix=$(tail -1 VERSION)
time=$1
# EXAMPLE:
# bash train.sh 60 [last|best](OPTIONAL)
# new_prefix=lw2_on1_on5_vz1
# pretrained_prefix=lw2_on1_on5
# time=70
# checkpoint=last|best
checkpoint=${2:-best}

id="${new_prefix//_/-}"
echo "id to be plugged in new training job:"
echo $id

# echo $pretrained_prefix
# echo $new_prefix
cp run.env.orig run.env
echo "">>run.env
if [ "$pretrained_prefix" = "$new_prefix" ]; then
    echo "DR_LOCAL_S3_PRETRAINED=False">>run.env
else
    echo "DR_LOCAL_S3_PRETRAINED=True">>run.env
    echo "DR_LOCAL_S3_PRETRAINED_PREFIX=$pretrained_prefix">>run.env
    echo "DR_LOCAL_S3_PRETRAINED_CHECKPOINT=$checkpoint">>run.env
fi
echo "DR_LOCAL_S3_MODEL_PREFIX=$new_prefix">>run.env

echo "============run.env file content============"
cat run.env
echo "============run.env file content============"

cd ..
echo "current directory:"
pwd

echo "./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} $time"
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} $time
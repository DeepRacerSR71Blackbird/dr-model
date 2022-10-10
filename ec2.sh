cd deepracer-templates
    5  ls
    6  cd custom-files/
    7  ls
    8  vi hyperparameters.json 
    9  ls
   10  vi readme
   11  ls
   12  vi hyperparameters.json 
   13  ls
   14  cp hyperparameters.json hyperparameters.json.default
   15  vi hyperparameters.json
   16  ls
   17  vi model_metadata_sac.json 
   18  ls
   19  cp model_metadata_sac.json model_metadata.json
   20  mv model_metadata_sac.json model_metadata_sac_continuous.json 
   21  ls
   22  vi model_metadata.json 
   23  ls
   24  wget https://raw.githubusercontent.com/aws-deepracer-community/deepracer-for-cloud/master/defaults/model_metadata.json model_metadata_discrete.json
   25  ls
   26  mv model_metadata.json.1 model_metadata_discrete.json
   27  ls
   28  vi reward_function.py
   29  ls
   30  vi reward_function.py
   31  ls -al reward_function.py
   32  vi reward_function.py
   33  vi run.env 
   34  ls
   35  ./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-0620-01 80 > train.log &
   36  cd ..
   37  ./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-0620-01 80 > train.log &
   38  tail -f train.log 
   39  ./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-0620-01 80 &> train.log &
   40  tail -f train.log 
   41  ls
   42  history

./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-0621-01 80 &> train-0621-01.log &
./add-access.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-XXXX-XX 99.62.7.149

# 1. change hyperparams as needed
# 2. change model prefix
# 3. change pretrained model prefix
id=0622-01
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-vz-${id} 140 &> train-${id}.log &
./add-access.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-whitelistip-vz-${id} 99.62.7.149

id=0923
./add-access.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-whitelistip-vz-${id} 99.62.7.149



# s3 bucket - s3://dfwdr-blackbird71-training-stack-1-bucket-bj6bdmt50m44/
# QUESTION: if i start two training with different env and hyperparam, will them affect each other?
# QUESTION: s3 log folder doesn't have anything

# 0621-01: start from scratch, speed from 1 to 2, used discount factor (df) of 0.9, too slow
# http://54.209.127.203:8080/
# vz-steer-speed-test2/

# 0621-02: start from scratch, speed from 1 to 2, with df=0.5 
# vz-steer-speed-1/
# http://44.201.161.232:8080/
# https://s3.console.aws.amazon.com/s3/buckets/dfwdr-blackbird71-training-stack-1-bucket-bj6bdmt50m44/DFWDR-blackbird71-training-job-vz-0621-02/logs/

# 0621-03: start from generic-5, speed from 1 to 4, with df=0.9
# vz-steer-speed-1-on-generic-5/
# http://3.91.242.137:8080/

# 0622-01
# start from generic-5, speed from 1 to 4, with df=0.9
# vz-steer-speed-1-on-generic-5/
# 54.89.60.31:8080

# 0622-02
# no speed reward, df=0.5, start from scratch, [0.5,4]
# vz-steer-1

# vz-steer-2
id=vz-steer-2
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} 80 &> logs/train-${id}.log &

./add-access.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-whitelistip-${id} 99.62.7.149
70.119.105.35

id=vz-generic-6
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} 80 &> logs/train-${id}.log &
# vz-steer-speed-test2: fresh new model
# 

id=xc7-on1-vz5
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} 70 &> logs/train-${id}.log &
+ aws cloudformation deploy --stack-name DFWDR-blackbird71-training-job-xc7-on1-vz4 --parameter-overrides ResourcesStackName=DFWDR-blackbird71-training-stack-1 TimeToLiveInMinutes=95 --template-file standard-instance.yaml

Waiting for changeset to be created..
Waiting for stack create/update to complete



Successfully created/updated stack - DFWDR-blackbird71-training-job-xc7-on1-vz4
++ aws cloudformation list-exports --query 'Exports[?Name=='\''DFWDR-blackbird71-training-job-xc7-on1-vz4-PublicIp'\''].Value' --no-paginate --output text
+ EC2_IP=52.204.192.192
+ echo 'Logs will upload every 2 minutes to https://s3.console.aws.amazon.com/s3/buckets/dfwdr-blackbird71-training-stack-1-bucket-bj6bdmt50m44/DFWDR-blackbird71-training-job-xc7-on1-vz4/logs/'
Logs will upload every 2 minutes to https://s3.console.aws.amazon.com/s3/buckets/dfwdr-blackbird71-training-stack-1-bucket-bj6bdmt50m44/DFWDR-blackbird71-training-job-xc7-on1-vz4/logs/
+ echo 'Training should start shortly on 52.204.192.192:8080'
Training should start shortly on 52.204.192.192:8080

id=xc6-on2-vz1
./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} 70 &> logs/train-${id}.log &

./add-access.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-whitelistip-vz-${id} 76.184.213.128
1. starbucks (mcdermott & watters) - 70.119.105.35
2. starbucks (eldorado & hardin) 76.184.213.128

[cloudshell-user@ip-10-1-137-230 deepracer-templates]$ id=xc7-on1-vz4
[cloudshell-user@ip-10-1-137-230 deepracer-templates]$ ./create-standard-instance.sh DFWDR-blackbird71-training-stack-1 DFWDR-blackbird71-training-job-${id} 95 &> logs/train-${id}.log &
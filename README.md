# dr-model template and training tracker
## clone model
git clone https://github.com/DeepRacerSR71Blackbird/dr-model.git
mv dr-model custom-files
## create your new branch
git checkout -b lw2_on1_on5_vz3
## copy or revise the following files to your preferred
cp ../custom-files-backup/model_metadata.json ./
cp ../custom-files-backup/hyperparameters.json ./
cp ../custom-files-backup/reward_function.py ./
##  confirm the changes for this new model training
[git diff|vimdiff] model_metadata, hyperparameters, reward_function
## Kick off training:
## train.sh [NEW_MODEL_PREFIX] [PRETRAINED_MODEL_PREFIX] [TRAINING_TIME] &> [LOG_PATH] %
bash ./train.sh lw2_on1_on5_vz3 lw2_on1_on5_vz2 70 &>../logs/train-lw2_on1_on5_vz3.log &
## check log
tail -f ../logs/train-lw2_on1_on5_vz3.log 
## NOTES
we recommend setting the same name for your git branch and $NEW_MODEL_PREFIX, as shown below
```
git checkout -b lw2_on1_on5_vz3
bash ./train.sh lw2_on1_on5_vz3 lw2_on1_on5_vz2 70 &>../logs/train-lw2_on1_on5_vz3.log &
```
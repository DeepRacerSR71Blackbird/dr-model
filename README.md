# dr-model template and training tracker
1. clone model
```
cd /PATH/TO/deepracer-templates
git clone https://github.com/DeepRacerSR71Blackbird/dr-model.git
mv dr-model custom-files
```
2. create your new branch
```
git checkout -b lw2_on1_on5_vz3
```
3. copy or revise the following files to your preferred
```
cp ../custom-files-backup/model_metadata.json ./
cp ../custom-files-backup/hyperparameters.json ./
cp ../custom-files-backup/reward_function.py ./
```
4.  confirm the changes for this new model training
```
[git diff|vimdiff] [model_metadata.json|hyperparameters.json|reward_function.py]
```
5. Kick off training:
  * Usage - ```bash train.sh [NEW_MODEL_PREFIX] [PRETRAINED_MODEL_PREFIX] [TRAINING_TIME] &> [LOG_PATH] &```
  * Example - ```bash ./train.sh lw2_on1_on5_vz3 lw2_on1_on5_vz2 70 &>../logs/train-lw2_on1_on5_vz3.log &```
6. check log
```
tail -f ../logs/train-lw2_on1_on5_vz3.log 
```
7. NOTES
we recommend setting the same name for your git branch and $NEW_MODEL_PREFIX, as shown below
```
git checkout -b lw2_on1_on5_vz3
bash ./train.sh lw2_on1_on5_vz3 lw2_on1_on5_vz2 70 &>../logs/train-lw2_on1_on5_vz3.log &
```
#!/bin/bash

# Copyright (C) 2019-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.


START_DIR=$PWD
echo "This shell script is meant to update the copyright in all CARMA related source files!"

read -sp 'Enter the full folder path: ' DIR
echo
echo 'Folder Path: ' $DIR
read -p 'Is this the correct folder? <y/n> ' prompt1

if [[ $prompt1 == "n" || $prompt1 == "N" || $prompt1 == "no" || $prompt1 == "No" ]]
then
  echo 'You answered' $prompt1', exiting.'
  return 0
fi

echo "Chosen folder: $DIR"
cd $DIR


read -sp 'Enter the current year (YYYY): ' YEAR
echo
echo "Entered Year: $YEAR"

read -p 'Is this the correct year? Would you like to proceed to update all the files recursively in the folder? <y/n> ' prompt2

if [[ $prompt2 == "n" || $prompt2 == "N" || $prompt2 == "no" || $prompt2 == "No" ]]
then
  echo 'You answered' $prompt2', exiting.'
  return 0
fi
    
echo "Please wait for the update to complete ..."

for OLD in `eval echo {2018..$(($YEAR-1))}`
do
  for NEW in `eval echo {$OLD..$(($YEAR-1))}`
  do
    if [[ $OLD == $NEW ]]
    then
      OLDYEARTEXT=$OLD
      NEWYEARTEXT=$OLD-$YEAR
    else   
      OLDYEARTEXT=$OLD-$NEW
      NEWYEARTEXT=$OLD-$YEAR
    fi
   
    OLDCOPYRIGHTTEXT=$"Copyright \(C\) $OLDYEARTEXT LEIDOS"
    NEWCOPYRIGHTTEXT=$"Copyright \(C\) $NEWYEARTEXT LEIDOS"

    echo "Updating the old copyright: $OLDCOPYRIGHTTEXT"
    echo "To the new copyright: $NEWCOPYRIGHTTEXT"

    find . -type f -exec sed -r -i "s/$OLDCOPYRIGHTTEXT/$NEWCOPYRIGHTTEXT/g" {} \;
  done
done

echo 'The copyright text has been updated in all the files. Please go to github to verify the changes are correct, and manually check it in for peer review. Thank you!'
cd $START_DIR

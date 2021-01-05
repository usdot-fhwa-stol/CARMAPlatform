#!/bin/bash

# Copyright (C) 2017-2021 LEIDOS.
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

source STOL_Developers.txt

START_DIR=$PWD
echo "This shell script is meant to add missing copyright info in all CARMA related source files"

read -p 'Do you want to write the copyrights? (Say no if you just want to see the authors of files who are missing copyrights) <y/n>: ' wprompt
if [[ $wprompt == "n" || $wprompt == "N" || $wprompt == "no" || $wprompt == "No" ]]
then
  echo 'You answered' $wprompt', not writing copyrights'
  export WRITE="no"
else
  echo 'You answered' $wprompt', copyrights will be written'
  export WRITE="yes"
fi

read -p 'Enter the full folder path: ' DIR
echo
echo 'Folder Path: ' $DIR
read -p 'Is this the correct folder? <y/n> ' prompt1

if [[ $prompt1 == "n" || $prompt1 == "N" || $prompt1 == "no" || $prompt1 == "No" ]]
then
  echo 'You answered' $prompt1', exiting.'
  cd $START_DIR
  return 0
fi

echo "Chosen folder: $DIR"
cd $DIR

if [[ $WRITE == "yes" ]]; then
  read -p 'Enter the year to update the copyrights to (YYYY): ' YEAR
  echo
  echo "Entered Year: $YEAR"

  read -p 'Is this the correct year? Would you like to proceed to update all the files recursively in the folder? <y/n> ' prompt2

  if [[ $prompt2 == "n" || $prompt2 == "N" || $prompt2 == "no" || $prompt2 == "No" ]]
  then
    echo 'You answered' $prompt2', exiting.'
    cd $START_DIR
    return 0
  fi
fi

# Define a function to return the copyright text for a file depending on its file type

function get_copyright_text {
  local extension="$1"
  local current_year="$2"
  local add_year="$3"
  if [ "$extension" == "cpp" ] || [ "$extension" == "hpp" ] || [ "$extension" == "h" ]; then
    if [[ "$current_year" == "$add_year" ]]; then
      copyright_text="\n\/*------------------------------------------------------------------------------\n* Copyright (C) $add_year LEIDOS.\n*\n* Licensed under the Apache License, Version 2.0 (the \"License\"); you may not\n* use this file except in compliance with the License. You may obtain a copy of\n* the License at\n*\n* http:\/\/www.apache.org\/licenses\/LICENSE-2.0\n*\n* Unless required by applicable law or agreed to in writing, software\n* distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n* License for the specific language governing permissions and limitations under\n* the License.\n\n------------------------------------------------------------------------------*\/\n"
    else
      copyright_text="\n\/*------------------------------------------------------------------------------\n* Copyright (C) $add_year-$current_year LEIDOS.\n*\n* Licensed under the Apache License, Version 2.0 (the \"License\"); you may not\n* use this file except in compliance with the License. You may obtain a copy of\n* the License at\n*\n* http:\/\/www.apache.org\/licenses\/LICENSE-2.0\n*\n* Unless required by applicable law or agreed to in writing, software\n* distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n* License for the specific language governing permissions and limitations under\n* the License.\n\n------------------------------------------------------------------------------*\/\n"
    fi
  elif [ "$extension" == "txt" ] || [ "$extension" == "py" ] || [ "$extension" == "sh" ]; then
    if [[ "$current_year" == "$add_year" ]]; then
      copyright_text="\n# Copyright (C) $add_year LEIDOS.\n#\n# Licensed under the Apache License, Version 2.0 (the \"License\"); you may not\n# use this file except in compliance with the License. You may obtain a copy of\n# the License at\n#\n# http:\/\/www.apache.org\/licenses\/LICENSE-2.0\n#\n# Unless required by applicable law or agreed to in writing, software\n# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n# License for the specific language governing permissions and limitations under\n# the License.\n"
    else
      copyright_text="\n# Copyright (C) $add_year-$current_year LEIDOS.\n#\n# Licensed under the Apache License, Version 2.0 (the \"License\"); you may not\n# use this file except in compliance with the License. You may obtain a copy of\n# the License at\n#\n# http:\/\/www.apache.org\/licenses\/LICENSE-2.0\n#\n# Unless required by applicable law or agreed to in writing, software\n# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n# License for the specific language governing permissions and limitations under\n# the License.\n"
    fi
  elif [ "$extension" == "test" ] || [ "$extension" == "launch" ]; then 
    if [[ "$current_year" == "$add_year" ]]; then
      copyright_text="\n<!-- Copyright (C) $add_year LEIDOS.\nLicensed under the Apache License, Version 2.0 (the \"License\"); you may not\nuse this file except in compliance with the License. You may obtain a copy of\nthe License at\nhttp:\/\/www.apache.org\/licenses\/LICENSE-2.0\nUnless required by applicable law or agreed to in writing, software\ndistributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\nWARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\nLicense for the specific language governing permissions and limitations under\nthe License.\n-->\n"
    else
      copyright_text="\n<!-- Copyright (C) $add_year-$current_year LEIDOS.\nLicensed under the Apache License, Version 2.0 (the \"License\"); you may not\nuse this file except in compliance with the License. You may obtain a copy of\nthe License at\nhttp:\/\/www.apache.org\/licenses\/LICENSE-2.0\nUnless required by applicable law or agreed to in writing, software\ndistributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\nWARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\nLicense for the specific language governing permissions and limitations under\nthe License.\n-->\n"
    fi
  fi
}

export -f get_copyright_text

# Define function to check each file for a copyright

function check_for_copyright {
    local file="$1"
    local current_year="$2"
    local date=$(git log --diff-filter=A --pretty=format:%ad --date=short -- $file) # Find the date the file was added
    local email=$(git log --diff-filter=A --pretty=format:%ae -- $file) # Find the email of the author who added the file
    local author=$(git log --diff-filter=A --pretty=format:%an -- $file) # Find the name of the author who added the file
    local extension="${file##*.}"
    if ! test -z "$date" ; then
        local add_year=${date:0:4}
        if ! egrep -q "Copyright" $file; then # Skip if the file has any copyright
          if [[ $DEVELOPERS =~ (^|[[:space:]])"$email"($|,[[:space:]]) ]] || [[ "$email" =~ ^[A-Za-z0-9._%+-]+@leidos\.com$ ]] ; then # Skip if the author email is not on the approved list (needs to be manually updated)
            echo " -- Found missing copyright in "$file", authored by "$author" ("$email")"
            get_copyright_text $extension $current_year $add_year

            if [[ $WRITE == "yes" ]]; then
              echo " --- Writing copyright"
              # Check if the files has a shebang (if so, put the copyright below it)
              if IFS= LC_ALL=C read -rN2 shebang < $file && [ "$shebang" = '#!' ]; then
                sed -i "2s/^/$copyright_text\n/" $file
              else
                sed -i "1s/^/$copyright_text\n/" $file
              fi
            fi

          else
            if ! [[ $KNOWN_NOT_DEVELOPERS =~ (^|[[:space:]])$email($|,[[:space:]]) ]]; then
              echo "$file has an author who is not a LEIDOS developer, "$author" ($email), skipping file."
            fi
          fi
          return 0
        fi
    else
        return 1
    fi
}

export -f check_for_copyright

# Find all files that do not have copyright that need it
echo "Searching files for missing copyrights, if the script skips over a STOL developer, add their email to engineering-tools/STOL_Developers.txt"

find . -type f \( -name "*.h" -o -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" -o -name "*.launch" -o -name "*.test" -o -name "*.sh" -o -name "*.py" \) -exec bash -c 'check_for_copyright "$@"' bash {} "$YEAR" \;
# find . -type f -exec bash -c 'check_for_copyright "$@"' bash {} "$YEAR" \;

echo "Done searching for missing copyrights"
cd $START_DIR
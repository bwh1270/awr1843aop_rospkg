#! /bin/bash

# Absolute path to this script.
SCRIPT=$(readlink -f $0)
# Absolute path this script is in.
SCRIPTPATH=`dirname $SCRIPT`

cd "$SCRIPTPATH"
#echo "$SCRIPTPATH"

# Export the environment variable
export SOURCE_PATH="/home/aims/woohyun/fmcw_radar/devel"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln aims_session.yml .tmuxinator.yml

# start tmuxinator
tmuxinator start -p aims_session.yml
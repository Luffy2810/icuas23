#!/bin/bash
# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln orb.yml .tmuxinator.yml

# start tmuxinator
#tmuxinator
export Bag_File=$1
tmuxinator start -p orb.yml

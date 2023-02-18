#############################################
## Stript to build the solution of exercise 1
#############################################

### Credentials #############
### Here put your credentials
token_user="teamXX"
token_code="XXXXXXXXXXXXXX"
##############################

## Don't touch this part
#########################
# creating the workspace
#########################
current_dir=$PWD
mkdir git-repos
cd git-repos
git clone https://$token_user:$token_code@gitlab.com/intro2ros_2022/$token_user/EXERCISE2/agitr_chapter3
git clone https://$token_user:$token_code@gitlab.com/intro2ros_2022/$token_user/EXERCISE2/agitr_chapter6
git clone https://gitioc.upc.edu/rostutorials/demoparameters.git
cd $current_dir
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s $current_dir/git-repos/* .
cd $current_dir/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build

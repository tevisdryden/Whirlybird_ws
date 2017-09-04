# whirlybird ROS workspace

This is the student workspace for the whirlybird. It contains all the necessary submodules
to run the whirlybird.  To install log into your caedm account, pull up a terminal and 
run the follwing commands

``` bash
cd ~
git clone https://magiccvs.byu.edu/gitlab/whirlybird/whirlybird_ws.git 
cd whirlybird_ws
git submodule update --init --recursive
catkin_make 
```

Now you will need to add some lines to your `~/.bashrc` in order to connect to the whirlybirds. This is done with
the following commands.

``` bash
echo "source ~/whirlybird_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/whirlybird_ws/whirlybird_lab.bash" >> ~/.bashrc
```

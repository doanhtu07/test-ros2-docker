# Virtual machines

So the structure and strategy are the same for both VMWare and VirtualBox

I use a technology called Vagrant, similar to Docker Compose, but for managing virtual machines

For Turtlebot4 with ROS 2, I recommend using VMWare because VirtualBox is just too laggy

The only tradeoff is that the process of installing VMWare is harder

## 1. How to use Vagrantfile

1. Install Vagrant

2. Follow this and install necessary dependencies for Vagrant and VMWare Fusion

- https://gist.github.com/manojkarthick/518ceb4569156ce2c95329b71f676bf0

- NOTE: For VirtualBox, you don't need to install any extra plugins

## 2. Run scripts

There are 2 cool scripts I already put inside `/scripts` folder inside `/src`

- The `prepare.sh` script will adjust `bashrc`

- The `setup.sh` script is an example of me connecting to 20 ROS 2 Turtlebot4 Discovery Servers

  - You can view it for learning experience

## Extra notes

### Virtual machine notes

- You should see Network Adapter set to bridged network on Wi-Fi or Autodetect

### Unbuntu notes

- Use Ctrl+Insert or Ctrl+Shift+C for copying and Shift+Insert or Ctrl+Shift+V for pasting text in the terminal in Ubuntu

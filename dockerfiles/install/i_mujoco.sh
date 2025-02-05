MUJOCO_VERSION_2=2.3.7

cd /root/install
wget https://github.com/deepmind/mujoco/releases/download/${MUJOCO_VERSION_2}/mujoco-${MUJOCO_VERSION_2}-linux-x86_64.tar.gz
tar -zxvf mujoco-${MUJOCO_VERSION_2}-linux-x86_64.tar.gz
wget https://github.com/deepmind/mujoco/releases/download/${MUJOCO_VERSION_3}/mujoco-${MUJOCO_VERSION_3}-linux-x86_64.tar.gz
tar -zxvf mujoco-${MUJOCO_VERSION_3}-linux-x86_64.tar.gz

gnome-terminal --window -e 'bash -c " source devel/setup.bash;roslaunch vins vins_rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source devel/setup.bash;rosrun vins vins_node ~/VINS/VINS_FUSION/src/VINS-Fusion-master/config/euroc/euroc_mono_imu_config.yaml; exec bash"' \
--tab -e 'bash -c "sleep 3; source devel/setup.bash;rosrun loop_fusion loop_fusion_node ~/VINS/VINS_FUSION/src/VINS-Fusion-master/config/euroc/euroc_mono_imu_config.yaml; exec bash"' \




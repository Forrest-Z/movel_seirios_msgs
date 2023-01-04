# 3D Mapping - Related Issue and Resources

## RTABMAP (& related)




-  RTABMap C&R Mapping Documentation

    https://docs.google.com/document/d/1rHxukPFND65j1e_1LFHeTUVwhPmrMVMDyud5do-kw74/edit



- RTABMap ICP
    https://docs.google.com/document/d/14aOgFkNCGjCuqjHODvdq8yg4fqrVX8deZrH3EhDbVVo/edit


- 3D Dynamic Mapping
    https://docs.google.com/document/d/1nXD76Wb-21lk2C4vIBKkJ9RaW0MsvqcWbVtfd4EYAWU/edit

## Odometry stuff for mapping


- Libpointmatcher vs FLOAM

    https://docs.google.com/document/d/1Tb53polAE0CUHTPESbS1ne9XJu8I0yz8QVS_qKQoT-o/edit

- FLOAM (research paper summary?)

    https://docs.google.com/document/d/1R0pMLNQNceaDTA2bsS5SmydyvJCU7nMekNFe9-WF2tU/edit


## Deprecated

- HDL Graph SLAM

    https://docs.google.com/document/d/1UGY5h2l8txuMW_W0p0VAVYCnh0phz3UJpc0YroSHGYY/edit


# Usage    

- Launch your 3d sensors i.e. 3d lidar.
- Go to the main docker-compose file (docker-compose.yaml). 

    - Set environment variable THREE_D_MAPPING: “true” under the section seirios-frontend.

    - Set entrypoint: bash -c 'source /ros_entrypoint.sh && roslaunch movel start_3d.launch' under the section seirios-ros.



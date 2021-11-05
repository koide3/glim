![GLIM](docs/logo2.png "GLIM Logo")

GLIM is a versatile and extensible range-based 3D mapping framework that is distinct from other existing mapping frameworks in several ways:

- ***Accuracy:*** The backend of GLIM is based on global matching cost minimization that enables to accurately retain the global consistency of the map compared to the conventional pose graph optimization. Optionally, GPU acceleration can be enabled to maximize the mapping speed and quality.
- ***Easy-to-use:*** GLIM offers an interactive map correction interface that enables the user to manually correct mapping failures and easily improve the mapping quality. It also provides a self-tuning mechanism that relieves the user from tedious parameter tuning.
- ***Versatility:*** As we eliminated sensor-specific processes, GLIM can be applied to any kind of range sensors including:
  - Spinning-type LiDAR (e.g., Velodyne HDL32e)
  - Non-repetitive scan LiDAR (e.g., Livox Avia)
  - Solid-state LiDAR (e.g., Intel Realsense L515)
  - RGB-D camera (e.g., Microsoft Azure Kinect)
- ***Extensibility:*** GLIM provides the global callback slot mechanism that allows the user to access and monitor the internal states of the mapping process and insert additional constraints to the factor graph.

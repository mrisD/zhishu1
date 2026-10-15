# Gazebo TCP auto calibration

- arm/tool: right / probe
- flange: R_hand_link
- method: four_point
- train samples: 4; independent validation: 3
- noise: gaussian
- TCP XYZ (flange, m): [0.0141737623, 4.54241712e-05, -0.299868962]
- TCP RPY (flange, rad): [-3.01108531, -0.0827965144, 0.1820099]
- orientation method: ABB TCP&Z,X
- ground truth (sim only, m): [0.015, 0, -0.3]
- fit RMS/MAX (mm): 0.211228 / 0.284941
- independent validation RMS/MAX (mm): 0.394533 / 0.416016
- TCP-to-truth error (mm): 0.837797
- orientation-to-truth error (deg): 0.642374
- independent frame holdout position RMS (mm): 0.317976
- independent frame holdout orientation RMS (deg): 0.0616953
- matrix condition: 11.1742

- collision-checked Cartesian pivot path planned/observed max (mm): 0.0735834 / 0.0755325
Results are not valid for the physical robot without a separate physical calibration.

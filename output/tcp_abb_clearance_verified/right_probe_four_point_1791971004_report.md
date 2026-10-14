# Gazebo TCP auto calibration

- arm/tool: right / probe
- flange: R_hand_link
- method: four_point
- train samples: 4; independent validation: 3
- noise: ideal
- TCP XYZ (flange, m): [0.0144975372, 4.92034937e-05, -0.299539232]
- TCP RPY (flange, rad): [-3.02625685, -0.0851442482, 0.178037051]
- orientation method: ABB TCP&Z,X
- ground truth (sim only, m): [0.015, 0, -0.3]
- fit RMS/MAX (mm): 0.108477 / 0.121417
- independent validation RMS/MAX (mm): 0.368456 / 0.543741
- TCP-to-truth error (mm): 0.683518
- orientation-to-truth error (deg): 0.419401
- independent frame holdout position RMS (mm): 0.238593
- independent frame holdout orientation RMS (deg): 0.0615458
- matrix condition: 11.1792

Results are not valid for the physical robot without a separate physical calibration.

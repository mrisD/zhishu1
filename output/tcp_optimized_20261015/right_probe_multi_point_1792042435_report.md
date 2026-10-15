# Gazebo TCP auto calibration

- arm/tool: right / probe
- flange: R_hand_link
- method: multi_point
- train samples: 12; independent validation: 3
- noise: ideal
- TCP XYZ (flange, m): [0.0149981661, -2.62420954e-06, -0.30000218]
- TCP RPY (flange, rad): [-3.01971971, -0.0813458199, 0.185080335]
- orientation method: ABB TCP&Z,X
- ground truth (sim only, m): [0.015, 0, -0.3]
- fit RMS/MAX (mm): 0.00582048 / 0.00882239
- independent validation RMS/MAX (mm): 0.00325412 / 0.00483975
- TCP-to-truth error (mm): 0.00387337
- orientation-to-truth error (deg): 0.327452
- independent frame holdout position RMS (mm): 0.287416
- independent frame holdout orientation RMS (deg): 0.0736522
- matrix condition: 8.16891

- collision-checked Cartesian pivot path planned/observed max (mm): 0.0734457 / 0.0746175
Results are not valid for the physical robot without a separate physical calibration.

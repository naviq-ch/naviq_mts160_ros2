# One reported track (the MTS160 always reports a left and a right track;
# they are identical when a single tape is under the sensor).
float32 position_mm      # lateral offset of the tape centreline from the sensor centre;
                         # left negative / right positive, 1 mm resolution, +/-80 mm
float32 angle_deg        # incidence angle of the tape, 1 deg resolution, +/-90 deg;
                         # sign convention per calibration.yaml / README

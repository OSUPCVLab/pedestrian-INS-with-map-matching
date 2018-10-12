function gyro_bias = compute_gyro_bias(gyroX, gyroY, gyroZ)

gyro_bias = [mean(gyroX) mean(gyroY) mean(gyroZ)]';

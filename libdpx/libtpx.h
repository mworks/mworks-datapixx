void TPxGetEyePosition_validation(float x[2], float y[2], float coeff_x[9][2], float coeff_y[9][2], float* eyeReturn);
void TPxGetEyePosition_lib(float coeff_x[9][2], float coeff_y[9][2], float* eyeReturn);
void TPxGetEyePosition_noCeoff(float* eyeReturn);

int TPxFinishCalibration_poly1(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2]);
int TPxFinishCalibration_poly1_returns_error(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x[2], float error_y[2]);
int TPxFinishCalibration_poly_general(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2]);

int TPxFinishCalibration_poly_general_3terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2]);
int TPxFinishCalibration_poly_general_4terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2]);
int TPxFinishCalibration_poly_general_5terms(float eye_data[25][8], float screen_data[25][2], float coeff_x[9][2], float coeff_y[9][2], float error_x_min[2], float error_y_min[2]);

void TPxSaveCoefficientsInTracker(float coeff_x[9][2], float coeff_y[9][2]);

void TPxSaveCoefficient_test();

void TPxReadCoefficient_test();
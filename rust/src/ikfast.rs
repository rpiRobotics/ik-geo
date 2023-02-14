use nalgebra::{ Matrix3, Vector3, Vector6 };

extern "C" {
    fn kuka_kr30l16_c(rotation: *const f64, translation: *const f64, q: *mut f64) -> usize;
}

pub fn kuka_kr30l16(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> Vec<Vector6<f64>> {
    let rotation_transposed = rotation.transpose();
    let rotation = rotation_transposed.as_slice();
    let translation = translation.as_slice();

    let mut q_data = [0.0; 6 * 8];
    let mut q: Vec<Vector6<f64>> = Vec::new();

    let solutions = unsafe {
        kuka_kr30l16_c(rotation.as_ptr(), translation.as_ptr(), q_data.as_mut_ptr())
    };

    for i in 0..solutions {
        q.push(Vector6::from_column_slice(&q_data[i * 6 .. (i + 1) * 6]));
    }

    q
}

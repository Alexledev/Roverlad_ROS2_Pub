#ifndef KALMAN_H
#define KALMAN_H

// --- Simple 1D Kalman filter (constant value model) ---
class Kalman1D {
public:
    float x = 0.0f;   // state (estimate)
    float P = 1.0f;   // estimate covariance
    float Q = 1e-3f;  // process noise (tune)
    float R = 1e-2f;  // measurement noise (tune)
    bool initialized = false;

    // timing
    static uint64_t last_time_ms;


    Kalman1D() = default;

    void init(float init_x, float init_P = 1.0f) {
        x = init_x;
        P = init_P;
        initialized = true;
    }

    // dt included in case user wants to scale Q by dt.
    float update(float z, float dt_s) {
        if (!initialized) {
            init(z, 1.0f);
            return x;
        }

        // Predict step (identity model: x_k = x_{k-1})
        // Increase uncertainty due to process noise.
        // scale Q with dt_s to make it time-aware (optional)
        float Q_scaled = Q * (dt_s > 0.0f ? dt_s : 1.0f);
        P += Q_scaled;

        // Update step
        float K = P / (P + R);       // Kalman gain
        x = x + K * (z - x);         // state update
        P = (1.0f - K) * P;          // covariance update

        return x;
    }

    static float getDeltaT(uint64_t now_ms) {
        // compute dt using steady clock        
        float dt_s = 0.0f;
        if (last_time_ms == 0) 
        {
            dt_s = 0.01f;
        } 
        else 
        {
            uint64_t diff = (now_ms >= last_time_ms) ? (now_ms - last_time_ms) : 0;
            dt_s = diff / 1000.0f;

            if (dt_s <= 0.0f) 
                dt_s = 0.001f; // guard
        }

        last_time_ms = now_ms;

        return dt_s;
    }


    // Optional setters for tuning
    void setProcessNoise(float q) { Q = q; }
    void setMeasurementNoise(float r) { R = r; }
};


#endif

"""Type stubs for the nanobind-compiled FFI surface."""

class ImuSample:
    rate_of_turn_x_rad_s: float
    rate_of_turn_y_rad_s: float
    rate_of_turn_z_rad_s: float
    accel_x_ms2: float
    accel_y_ms2: float
    accel_z_ms2: float
    timestamp_ms: int
    def __init__(self) -> None: ...

class GnssSample:
    heading_deg: float
    heading_variance_deg2: float
    valid: bool
    timestamp_ms: int
    def __init__(self) -> None: ...

class FusionOutput:
    heading_deg: float
    heading_variance_deg2: float
    timestamp_ms: int

class Config:
    q_heading_deg2: float
    q_bias_deg2_s2: float
    p0_heading_deg2: float
    p0_bias_deg2_s2: float
    def __init__(self) -> None: ...

class TinyEkfFilter:
    def __init__(self, cfg: Config) -> None: ...
    def predict(self, imu: ImuSample) -> None: ...
    def update(self, gnss: GnssSample) -> None: ...
    def output(self) -> FusionOutput: ...

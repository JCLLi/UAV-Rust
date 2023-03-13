use protocol::WorkingModes;

//Mode switch function for safe mode
pub fn switch(new: WorkingModes) -> WorkingModes{
    match new {
        WorkingModes::PanicMode => new,
        WorkingModes::ManualMode => new,
        WorkingModes::YawControlMode => new,
        WorkingModes::CalibrationMode => new,
        _ => WorkingModes::SafeMode,
    }
}
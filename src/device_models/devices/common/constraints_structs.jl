struct DeviceDurationConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    duration_data::PSI.UpDown
    initial_duration::Tuple{PSI.InitialCondition, PSI.InitialCondition}
    initial_outage::PSI.InitialCondition
    multiplier::Float64
    timeseries::Vector{Float64}
end

get_component_name(d::DeviceDurationConstraintInfo) = d.component_name
get_duration_data(d::DeviceDurationConstraintInfo) = d.duration_data
get_initial_duration(d::DeviceDurationConstraintInfo) = d.initial_duration
get_initial_outage(d::DeviceDurationConstraintInfo) = d.initial_outage
get_multiplier(d::DeviceDurationConstraintInfo) = d.multiplier
get_timeseries(d::DeviceDurationConstraintInfo) = d.timeseries

struct DeviceOutageConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    initial_condition::Union{Nothing, PSI.InitialCondition}
    multiplier::Float64
    timeseries::Vector{Float64}
end

get_component_name(d::DeviceOutageConstraintInfo) = d.component_name
get_initial_condition(d::DeviceOutageConstraintInfo) = d.initial_condition
get_multiplier(d::DeviceOutageConstraintInfo) = d.multiplier
get_timeseries(d::DeviceOutageConstraintInfo) = d.timeseries

struct DeviceOutageRampConstraintInfo <: PSI.AbstractRampConstraintInfo
    component_name::String
    limits::PSI.MinMax
    ic_power::PSI.InitialCondition
    ic_outage::PSI.InitialCondition
    ramp_limits::PSI.UpDown
    additional_terms_ub::Vector{Symbol}
    additional_terms_lb::Vector{Symbol}
end

function DeviceOutageRampConstraintInfo(
    component_name::String,
    limits::PSY.Min_Max,
    ic_power::PSI.InitialCondition,
    ic_outage::PSI.InitialCondition,
    ramp_limits::PSI.UpDown,
)
    return DeviceOutageRampConstraintInfo(
        component_name,
        limits,
        ic_power,
        ic_outage,
        ramp_limits,
        Vector{Symbol}(),
        Vector{Symbol}(),
    )
end

function DeviceOutageRampConstraintInfo(
    component_name::String,
    ic_power::PSI.InitialCondition,
    ic_outage::PSI.InitialCondition,
)
    return DeviceOutageRampConstraintInfo(
        component_name,
        (min = -Inf, max = Inf),
        ic_power,
        ic_outage,
        (up = Inf, down = Inf),
    )
end

get_limits(d::DeviceOutageRampConstraintInfo) = d.limits
get_ic_power(d::DeviceOutageRampConstraintInfo) = d.ic_power
get_ic_outage(d::DeviceOutageRampConstraintInfo) = d.ic_outage
get_ramp_limits(d::DeviceOutageRampConstraintInfo) = d.ramp_limits
get_additional_terms_ub(d::DeviceOutageRampConstraintInfo) = d.additional_terms_ub
get_additional_terms_lb(d::DeviceOutageRampConstraintInfo) = d.additional_terms_lb

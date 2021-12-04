function PSI.calculate_ic_quantity(
    ::PSI.ICKey{InitialOutageStatus, T},
    ic::PSI.InitialCondition,
    var_value::Float64,
    simulation_cache::Dict{<:PSI.CacheKey, PSI.AbstractCache},
    elapsed_period::Dates.Period,
) where {T <: PSY.Component}
    current_status = isapprox(var_value, 0.0, atol = PSI.ABSOLUTE_TOLERANCE) ? 0.0 : 1.0
    return current_status
end

function _make_initial_condition_outage_status(
    container,
    device::T,
    value,
    cache = nothing,
) where {T <: PSY.Component}
    return PSI.InitialCondition(device, _get_ref_outage_status(T, container), value, cache)
end

function _get_ref_outage_status(
    ::Type{T},
    container::PSI.InitialConditions,
) where {T <: PSY.Component}
    return PSI.UpdateRef{JuMP.VariableRef}(T, OUTAGE)
end

function _get_outage_initial_value(
    d::PSY.Component,
    key::PSI.ICKey,
    ::PSI.AbstractDeviceFormulation,
    ::Nothing,
)
    return 1.0
end

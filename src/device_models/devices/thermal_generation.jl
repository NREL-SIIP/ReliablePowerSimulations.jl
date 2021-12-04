abstract type AbstractThermalOutageDispatchFormulation <:
              PSI.AbstractThermalDispatchFormulation end
struct ThermalStandardUCOutages <: PSI.AbstractStandardUnitCommitment end
struct ThermalBasicUCOutages <: PSI.AbstractStandardUnitCommitment end
struct ThermalDispatchOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end

############## AuxiliaryOnVariable, ThermalGen ####################
PSI.get_variable_binary(::AuxiliaryOnVariable, ::Type{<:PSY.ThermalGen}, _) = false
PSI.get_variable_initial_value(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) =
    PSY.get_status(d) ? 1.0 : 0.0

PSI.get_variable_lower_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 1.0

############## OutageVariable, ThermalGen ####################
PSI.get_variable_binary(::OutageVariable, ::Type{<:PSY.ThermalGen}, _) = false
PSI.get_variable_initial_value(::OutageVariable, d::PSY.ThermalGen, _) =
    PSY.get_status(d) ? 1.0 : 0.0

PSI.get_variable_lower_bound(::OutageVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::OutageVariable, d::PSY.ThermalGen, _) = 1.0

######## CONSTRAINTS ############

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{<:PSI.VariableType},
    ::Type{T},
    ::Type{<:PSI.AbstractThermalFormulation},
    ::Type{<:PM.AbstractPowerModel},
    feedforward::SemiContinuousOutagesFF,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {T <: PSY.ThermalGen}
    return PSI.DeviceRangeConstraintSpec()
end

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{PSI.ActivePowerVariable},
    ::Type{T},
    ::Type{S},
    ::Type{<:PSI.PM.AbstractPowerModel},
    feedforward::Nothing,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {
    T <: PSY.ThermalGen,
    S <: Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages},
}
    return PSI.DeviceRangeConstraintSpec(;
        range_constraint_spec = PSI.RangeConstraintSpec(;
            constraint_name = PSI.make_constraint_name(
                PSI.RangeConstraint,
                PSI.ActivePowerVariable,
                T,
            ),
            variable_name = PSI.make_variable_name(PSI.ActivePowerVariable, T),
            limits_func = x -> (min = 0.0, max = PSY.get_active_power_limits(x).max),
            constraint_func = PSI.device_range!,
            constraint_struct = PSI.DeviceRangeConstraintInfo,
        ),
        custom_optimization_container_func = custom_active_power_constraints!,
    )
end

function custom_active_power_constraints!(
    optimization_container::PSI.OptimizationContainer,
    ::PSI.IS.FlattenIteratorWrapper{T},
    ::Type{<:AbstractThermalOutageDispatchFormulation},
) where {T <: PSY.ThermalGen}
    var_key = PSI.make_variable_name(PSI.ActivePowerVariable, T)
    variable = PSI.get_variable(optimization_container, var_key)
    # If the variable was a lower bound != 0, not removing the LB can cause infeasibilities
    for v in variable
        if JuMP.has_lower_bound(v)
            JuMP.set_lower_bound(v, 0.0)
        end
    end
end

function _get_data_for_rocc_outage(
    optimization_container::PSI.OptimizationContainer,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    resolution = PSI.model_resolution(optimization_container)
    if resolution > Dates.Minute(1)
        minutes_per_period = Dates.value(Dates.Minute(resolution))
    else
        @warn("Not all formulations support under 1-minute resolutions. Exercise caution.")
        minutes_per_period = Dates.value(Dates.Second(resolution)) / 60
    end

    initial_conditions_power =
        PSI.get_initial_conditions(optimization_container, PSI.DevicePower, T)
    initial_conditions_outage =
        PSI.get_initial_conditions(optimization_container, InitialOutageStatus, T)
    lenght_devices_power = length(initial_conditions_power)
    lenght_devices_outage = length(initial_conditions_outage)
    @assert lenght_devices_power == lenght_devices_outage
    data = Vector{DeviceOutageRampConstraintInfo}(undef, lenght_devices_power)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_power)
        g = ic.device
        @assert g == initial_conditions_outage[ix].device
        name = PSY.get_name(g)
        ramp_limits = PSY.get_ramp_limits(g)
        if !(ramp_limits === nothing)
            p_lims = PSY.get_active_power_limits(g)
            max_rate = abs(p_lims.min - p_lims.max) / minutes_per_period
            if (ramp_limits.up >= max_rate) & (ramp_limits.down >= max_rate)
                @debug "Generator $(name) has a nonbinding ramp limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ramp = (
                up = ramp_limits.up * minutes_per_period,
                down = ramp_limits.down * minutes_per_period,
            )
            data[idx] = DeviceOutageRampConstraintInfo(
                name,
                p_lims,
                ic,
                initial_conditions_outage[ix],
                ramp,
            )
        end
    end
    if idx < lenght_devices_power
        deleteat!(data, (idx + 1):lenght_devices_power)
    end
    return data
end

function ramp_constraints!(
    optimization_container::PSI.OptimizationContainer,
    ::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages},
    S <: PSI.PM.AbstractPowerModel,
}
    data = _get_data_for_rocc_outage(optimization_container, T)
    if !isempty(data)
        # Here goes the reactive power ramp limits when versions for AC and DC are added
        for r in data
            PSI.add_device_services!(r, r.ic_power.device, model)
        end
        device_mixedinteger_rateofchange_outages!(
            optimization_container,
            data,
            PSI.make_constraint_name(PSI.RAMP, T),
            (
                PSI.make_variable_name(PSI.ActivePowerVariable, T),
                PSI.make_variable_name(PSI.StartVariable, T),
                PSI.make_variable_name(PSI.StopVariable, T),
            ),
            PSI.UpdateRef{T}(OUTAGE, "outage"),
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end

function ramp_constraints!(
    optimization_container::PSI.OptimizationContainer,
    ::PSI.IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    D <: PSI.AbstractThermalDispatchFormulation,
    S <: PSI.PM.AbstractPowerModel,
}
    data = _get_data_for_rocc_outage(optimization_container, T)
    if !isempty(data)
        for r in data
            PSI.add_device_services!(r, r.ic_power.device, model)
        end
        device_linear_rateofchange_outages!(
            optimization_container,
            data,
            PSI.make_constraint_name(PSI.RAMP, T),
            PSI.make_variable_name(PSI.ActivePowerVariable, T),
            PSI.UpdateRef{T}(OUTAGE, "outage"),
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end

function _get_data_for_tdc_outages(
    optimization_container::PSI.OptimizationContainer,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    resolution = PSI.model_resolution(optimization_container)
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    initial_conditions_on = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(PSI.InitialTimeDurationOn, T),
    )
    initial_conditions_off = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(PSI.InitialTimeDurationOff, T),
    )
    initial_conditions_outage = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(InitialOutageStatus, T),
    )
    lenght_devices_on = length(initial_conditions_on)
    lenght_devices_off = length(initial_conditions_off)
    lenght_devices_outage = length(initial_conditions_outage)
    @assert lenght_devices_off == lenght_devices_on == lenght_devices_outage
    data = Vector{DeviceDurationConstraintInfo}(undef, lenght_devices_on)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_on)
        g = ic.device
        @assert g == initial_conditions_off[ix].device
        @assert g == initial_conditions_outage[ix].device

        time_limits = PSY.get_time_limits(g)
        name = PSY.get_name(g)
        if !(time_limits === nothing)
            if (time_limits.up <= fraction_of_hour) & (time_limits.down <= fraction_of_hour)
                @debug "Generator $(name) has a nonbinding time limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            up_val = round(time_limits.up * steps_per_hour, RoundUp)
            down_val = round(time_limits.down * steps_per_hour, RoundUp)
            data[idx] = DeviceDurationConstraintInfo(
                name,
                (up = up_val, down = down_val),
                (ic, initial_conditions_off[ix]),
                initial_conditions_outage[ix],
                1.0,
                PSI.get_time_series(optimization_container, g, "outage"),
            )
        end
    end
    if idx < lenght_devices_on
        deleteat!(data, (idx + 1):lenght_devices_on)
    end
    return data
end

function time_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, U},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
    U <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages},
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    constraint_infos = _get_data_for_tdc_outages(optimization_container, T)
    forecast_label = "outage"
    if !(isempty(constraint_infos))
        if parameters
            device_duration_parameters_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_duration_look_ahead_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
            )
        end
    else
        @warn "Data doesn't contain generators with time-up/down limits, consider adjusting your formulation"
    end
    return
end

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
    D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages},
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(InitialOutageStatus, T),
    )
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in initial_conditions
        device = ic.device
        name = PSI.get_name(device)
        info = DeviceOutageConstraintInfo(
            name,
            ic,
            1.0,
            PSI.get_time_series(optimization_container, device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        if parameters
            device_outage_parameter!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                ),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
                PSI.make_variable_name(OutageVariable, T),
            )
        else
            device_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                ),
                PSI.make_variable_name(OutageVariable, T),
            )
        end
    else
        @warn "Data doesn't contain generators with initial condition for outage status, consider adjusting your formulation"
    end

    return
end

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
    D <: AbstractThermalOutageDispatchFormulation,
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(InitialOutageStatus, T),
    )
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in initial_conditions
        device = ic.device
        name = PSI.get_name(device)
        info = DeviceOutageConstraintInfo(
            name,
            ic,
            1.0,
            PSI.get_time_series(optimization_container, device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        if parameters
            device_outage_ub_parameter!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ACTIVE_POWER, T),
                PSI.make_variable_name(OutageVariable, T),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_outage_ub!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ACTIVE_POWER, T),
                PSI.make_variable_name(OutageVariable, T),
            )
        end
    else
        @warn "Data doesn't contain generators with initial condition for outage status, consider adjusting your formulation"
    end

    return
end

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
    D <: Union{ThermalDispatchOutages, ThermalRampLimitedOutages},
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    forecast_label = "outage"
    time_steps = PSI.model_time_steps(optimization_container)
    var_outage =
        PSI.get_variable(optimization_container, PSI.make_variable_name(OutageVariable, T))
    name_outage =
        PSI.make_constraint_name(OUTAGE, T)
    set_names = [PSY.get_name(d) for d in devices]
    con_outage =
        PSI.add_cons_container!(optimization_container, name_outage, set_names, time_steps)
    if parameters
        container_outage = PSI.get_parameter_container(
            optimization_container,
            PSI.UpdateRef{T}(OUTAGE, forecast_label),
        )
        param = PSI.get_parameter_array(container_outage)
        for device in devices, t in time_steps
            name = PSY.get_name(device)
            con_outage[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                var_outage[name, t] == param[name, t]
            )
        end
    else
        for device in devices, t in time_steps
            name = PSY.get_name(device)
            timeseries = PSI.get_time_series(optimization_container, device, forecast_label)
            con_outage[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                var_outage[name, t] == timeseries[t]
            )
        end
    end
    return
end

function add_outage_parameter!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
    D <: AbstractThermalOutageDispatchFormulation,
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    time_steps = PSI.model_time_steps(optimization_container)
    set_names = [PSY.get_name(d) for d in devices]
    forecast_label = "outage"
    container_outage = PSI.add_param_container!(
        optimization_container,
        PSI.UpdateRef{T}(OUTAGE, forecast_label),
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for d in devices, t in time_steps
        name = PSY.get_name(d)
        ts_vector = PSI.get_time_series(optimization_container, d, forecast_label)
        param[name, t] = PJ.add_parameter(optimization_container.JuMPmodel, ts_vector[t])
        multiplier[name, t] = 1.0
    end
    return
end

function PSI.initial_conditions!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: AbstractThermalOutageDispatchFormulation}
    PSI.output_initial_condition!(optimization_container, devices, formulation)
    outage_status_initial_condition!(optimization_container, devices, formulation)
    return
end

function PSI.initial_conditions!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    PSI.status_initial_condition!(optimization_container, devices, formulation)
    PSI.output_initial_condition!(optimization_container, devices, formulation)
    PSI.duration_initial_condition!(optimization_container, devices, formulation)
    outage_status_initial_condition!(optimization_container, devices, formulation)
    return
end

function outage_status_initial_condition!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::D,
) where {T <: PSY.ThermalGen, D <: PSI.AbstractThermalFormulation}
    PSI._make_initial_conditions!(
        optimization_container,
        devices,
        D(),
        nothing,
        PSI.ICKey(InitialOutageStatus, T),
        _make_initial_condition_outage_status,
        _get_outage_initial_value,
    )

    return
end

function PSI.cost_function!(
    optimization_container::PSI.OptimizationContainer,
    devices::PSI.IS.FlattenIteratorWrapper{T},
    ::PSI.DeviceModel{T, S},
    ::Type{<:PSI.PM.AbstractPowerModel},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages},
}
    no_min_spec = PSI.AddCostSpec(;
        variable_type = PSI.ActivePowerVariable,
        component_type = T,
        has_status_variable = PSI.has_on_variable(optimization_container, T),
        has_status_parameter = PSI.has_on_parameter(optimization_container, T),
        variable_cost = PSY.get_variable,
        fixed_cost = PSY.get_fixed,
    )
    resolution = PSI.model_resolution(optimization_container)
    dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
    for g in devices
        component_name = PSY.get_name(g)
        op_cost = PSY.get_operation_cost(g)
        cost_component = PSY.get_variable(op_cost)
        if isa(cost_component, PSY.VariableCost{Array{Tuple{Float64, Float64}, 1}})
            @debug "PWL cost function detected for device $(component_name) using ThermalDispatchNoMin"
            slopes = PSY.get_slopes(cost_component)
            if any(slopes .< 0) || !PSI.pwlparamcheck(cost_component)
                throw(
                    IS.InvalidValue(
                        "The PWL cost data provided for generator $(PSY.get_name(g)) is not compatible with a No Min Cost.",
                    ),
                )
            end
            if slopes[1] != 0.0
                @debug "PWL has no 0.0 intercept for generator $(PSY.get_name(g))"
                # adds a first intercept a x = 0.0 and Y below the intercept of the first tuple to make convex equivalent
                first_pair = PSY.get_cost(cost_component)[1]
                cost_function_data = deepcopy(cost_component.cost)
                intercept_point = (0.0, first_pair[2] - PSI.COST_EPSILON)
                cost_function_data = vcat(intercept_point, cost_function_data)
                # @assert PSI.slope_convexity_check(slopes)
            else
                cost_function_data = cost_component.cost
            end
            time_steps = PSI.model_time_steps(optimization_container)
            for t in time_steps
                gen_cost = PSI.pwl_gencost_linear!(
                    optimization_container,
                    no_min_spec,
                    component_name,
                    cost_function_data,
                    t,
                )
                PSI.add_to_cost_expression!(
                    optimization_container,
                    no_min_spec.multiplier * gen_cost * dt,
                )
            end
        else
            PSI.add_to_cost!(optimization_container, no_min_spec, op_cost, g)
        end
    end
    return
end

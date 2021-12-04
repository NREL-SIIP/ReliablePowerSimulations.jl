"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        OutageVariable,
        devices,
        ThermalStandardUCOutages(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalStandardUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    time_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        OutageVariable,
        devices,
        ThermalStandardUCOutages(),
    )
    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalStandardUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    time_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalBasicUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        OutageVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalBasicUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

"""
This function creates the model for a full thermal dispatch formulation depending on combination of devices, device_formulation and system_formulation
"""
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, ThermalBasicUCOutages},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.OnVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StartVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.StopVariable,
        devices,
        ThermalBasicUCOutages(),
    )
    PSI.add_variables!(
        optimization_container,
        OutageVariable,
        devices,
        ThermalBasicUCOutages(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, ThermalBasicUCOutages())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.commitment_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))

    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, D},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractThermalOutageDispatchFormulation,
    S <: PSI.PM.AbstractPowerModel,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(optimization_container, PSI.ReactivePowerVariable, devices, D())
    PSI.add_variables!(optimization_container, AuxiliaryOnVariable, devices, D())
    PSI.add_variables!(optimization_container, OutageVariable, devices, D())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, D())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    add_outage_parameter!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, D},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractThermalOutageDispatchFormulation,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(optimization_container, AuxiliaryOnVariable, devices, D())
    PSI.add_variables!(optimization_container, OutageVariable, devices, D())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, D())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    add_outage_parameter!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PSI.PM.AbstractPowerModel,
    V <: ThermalRampLimitedOutages,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, PSI.ReactivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, AuxiliaryOnVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    add_outage_parameter!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractActivePowerModel,
    V <: ThermalRampLimitedOutages,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, AuxiliaryOnVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    add_outage_parameter!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PSI.PM.AbstractPowerModel, V <: ThermalNoMinOutages}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, PSI.ReactivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {T <: PSY.ThermalGen, S <: PM.AbstractActivePowerModel, V <: ThermalNoMinOutages}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PSI.PM.AbstractPowerModel,
    V <: ThermalNoMinRampLimitedOutages,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, PSI.ReactivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{T, V},
    ::Type{S},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractActivePowerModel,
    V <: ThermalNoMinRampLimitedOutages,
}
    devices = PSI.get_available_components(T, sys)

    if !PSI.validate_available_devices(T, devices)
        return
    end

    # Variables
    PSI.add_variables!(optimization_container, PSI.ActivePowerVariable, devices, V())
    PSI.add_variables!(optimization_container, OutageVariable, devices, V())

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, V())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    outage_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    ramp_constraints!(optimization_container, devices, model, S, PSI.get_feedforward(model))
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Cost Function
    PSI.cost_function!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

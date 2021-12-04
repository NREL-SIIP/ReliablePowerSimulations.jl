function device_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
    outage_variable::Symbol,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varon = PSI.get_variable(optimization_container, var_names[1])
    varstop = PSI.get_variable(optimization_container, var_names[2])
    varstart = PSI.get_variable(optimization_container, var_names[3])
    var_outage = PSI.get_variable(optimization_container, outage_variable)
    name_on = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "on")
    name_off = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "off")
    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_outage =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)
    con_stop =
        PSI.add_cons_container!(optimization_container, name_off, set_names, time_steps)
    con_start =
        PSI.add_cons_container!(optimization_container, name_on, set_names, time_steps)

    for cont in constraint_info
        name = PSI.get_component_name(cont)
        con_stop[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstop[name, 1] >=
            (cont.initial_condition.value - cont.timeseries[1]) * varon[name, 1]
        )
        con_start[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstart[name, 1] <= cont.timeseries[1]
        )
        con_outage[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            var_outage[name, 1] == cont.timeseries[1]
        )
        for t in time_steps[2:end]
            con_stop[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] >=
                (cont.timeseries[t - 1] - cont.timeseries[t]) * varon[name, t]
            )
            con_start[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] <= cont.timeseries[t]
            )
            con_outage[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                var_outage[name, t] == cont.timeseries[t]
            )
        end
    end
    return
end

function device_outage_parameter!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
    param_reference::PSI.UpdateRef,
    outage_variable::Symbol,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varon = PSI.get_variable(optimization_container, var_names[1])
    varstop = PSI.get_variable(optimization_container, var_names[2])
    varstart = PSI.get_variable(optimization_container, var_names[3])
    var_outage = PSI.get_variable(optimization_container, outage_variable)
    name_on = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "on")
    name_start = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "start")
    name_stop = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "stop")

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_on = PSI.add_cons_container!(optimization_container, name_on, set_names, time_steps)
    con_outage =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)
    con_stop =
        PSI.add_cons_container!(optimization_container, name_stop, set_names, time_steps)
    con_start =
        PSI.add_cons_container!(optimization_container, name_start, set_names, time_steps)

    container_outage = PSI.add_param_container!(
        optimization_container,
        param_reference,
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for cont in constraint_info
        name = PSI.get_component_name(cont)
        param[name, 1] =
            PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[1])
        multiplier[name, 1] = cont.multiplier
        ic_outage = PSI.get_value(get_initial_condition(cont))
        varz = JuMP.@variable(
            optimization_container.JuMPmodel,
            base_name = "outage_z_{$(name), 1}"
        )
        vary = JuMP.@variable(
            optimization_container.JuMPmodel,
            base_name = "outage_y_{$(name), 1}"
        )
        JuMP.@constraint(optimization_container.JuMPmodel, varz <= varon[name, 1])
        JuMP.@constraint(optimization_container.JuMPmodel, vary <= varon[name, 1])

        JuMP.@constraint(optimization_container.JuMPmodel, varz <= ic_outage) # 
        JuMP.@constraint(optimization_container.JuMPmodel, vary <= param[name, 1])

        JuMP.@constraint(
            optimization_container.JuMPmodel,
            varz >= ic_outage + varon[name, 1] - 1.0
        )
        JuMP.@constraint(
            optimization_container.JuMPmodel,
            vary >= param[name, 1] + varon[name, 1] - 1.0
        )

        con_on[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varon[name, 1] <= param[name, 1]
        )
        con_start[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstart[name, 1] <= param[name, 1]
        )
        con_stop[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstop[name, 1] >= varz - vary
        )
        con_outage[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            var_outage[name, 1] == param[name, 1]
        )

        for t in time_steps[2:end]
            param[name, t] =
                PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[t])
            multiplier[name, t] = cont.multiplier
            varz = JuMP.@variable(
                optimization_container.JuMPmodel,
                base_name = "outage_z_{$(name), $(t)}"
            )
            vary = JuMP.@variable(
                optimization_container.JuMPmodel,
                base_name = "outage_y_{$(name), $(t)}"
            )
            JuMP.@constraint(optimization_container.JuMPmodel, varz <= varon[name, t])
            JuMP.@constraint(optimization_container.JuMPmodel, vary <= varon[name, t])

            JuMP.@constraint(optimization_container.JuMPmodel, varz <= param[name, t - 1])
            JuMP.@constraint(optimization_container.JuMPmodel, vary <= param[name, t])

            JuMP.@constraint(
                optimization_container.JuMPmodel,
                varz >= param[name, t - 1] + varon[name, t] - 1.0
            )
            JuMP.@constraint(
                optimization_container.JuMPmodel,
                vary >= param[name, t] + varon[name, t] - 1.0
            )
            con_stop[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] >= varz - vary
            )
            con_start[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] <= param[name, t]
            )
            con_on[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varon[name, t] <= param[name, t]
            )
            con_outage[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                var_outage[name, t] == param[name, t]
            )
        end
    end
    return
end

function device_outage_ub!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_name::Symbol,
    outage_variable::Symbol,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varp = PSI.get_variable(optimization_container, var_name)
    var_outage = PSI.get_variable(optimization_container, outage_variable)
    name_outage = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "param")
    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    constraint =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)
    con_outage =
        PSI.add_cons_container!(optimization_container, name_outage, set_names, time_steps)
    for cont in constraint_info, t in time_steps
        name = PSI.get_component_name(cont)
        constraint[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varp[name, t] <= (cont.timeseries[t]) * PSI.M_VALUE
        )
        con_outage[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            var_outage[name, t] == cont.timeseries[t]
        )
    end
    return
end

function device_outage_ub_parameter!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_name::Symbol,
    outage_variable::Symbol,
    param_reference::PSI.UpdateRef,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varp = PSI.get_variable(optimization_container, var_name)
    var_outage = PSI.get_variable(optimization_container, outage_variable)
    name_outage = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "param")
    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    constraint =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)
    con_outage =
        PSI.add_cons_container!(optimization_container, name_outage, set_names, time_steps)
    container_outage = PSI.add_param_container!(
        optimization_container,
        param_reference,
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for cont in constraint_info, t in time_steps
        name = PSI.get_component_name(cont)
        param[name, t] =
            PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[t])
        multiplier[name, t] = cont.multiplier
        constraint[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varp[name, t] <= param[name, t] * PSI.M_VALUE
        )
        con_outage[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            var_outage[name, t] == param[name, t]
        )
    end
    return
end

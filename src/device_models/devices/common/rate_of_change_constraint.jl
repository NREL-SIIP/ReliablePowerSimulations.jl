function device_linear_rateofchange_outages!(
    optimization_container::PSI.OptimizationContainer,
    rate_data::Vector{DeviceOutageRampConstraintInfo},
    cons_name::Symbol,
    var_name::Symbol,
    update_ref::PSI.UpdateRef,
)
    parameters = PSI.model_has_parameters(optimization_container)
    time_steps = PSI.model_time_steps(optimization_container)
    up_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "up")
    down_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "dn")

    variable = PSI.get_variable(optimization_container, var_name)

    set_name = [PSI.get_component_name(r) for r in rate_data]
    con_up = PSI.add_cons_container!(optimization_container, up_name, set_name, time_steps)
    con_down =
        PSI.add_cons_container!(optimization_container, down_name, set_name, time_steps)
    container_outage = PSI.get_parameter_container(optimization_container, update_ref)
    outage_parameter = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for r in rate_data
        name = PSI.get_component_name(r)
        ic_power = PSI.get_value(get_ic_power(r))
        ic_outage = PSI.get_value(get_ic_outage(r))
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, PJ.ParameterRef)) || !parameters
        expression_ub = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(optimization_container, val)[name, 1],
            )
        end
        con_up[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            expression_ub - ic_power <=
            r.ramp_limits.up + (r.limits.min - ic_outage * r.limits.min) + r.limits.max
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(optimization_container, val)[name, 1],
                -1.0,
            )
        end
        con_down[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            ic_power - expression_lb <=
            r.ramp_limits.down +
            (r.limits.max - outage_parameter[name, 1] * r.limits.max) +
            r.limits.max
        )
    end

    for t in time_steps[2:end], r in rate_data
        name = PSI.get_component_name(r)
        expression_ub = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(optimization_container, val)[name, t],
            )
        end
        con_up[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            expression_ub - variable[name, t - 1] <=
            r.ramp_limits.up +
            (r.limits.min - outage_parameter[name, t - 1] * r.limits.min)
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(optimization_container, val)[name, t],
                -1.0,
            )
        end
        con_down[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            variable[name, t - 1] - expression_lb <=
            r.ramp_limits.down + (r.limits.max - outage_parameter[name, t] * r.limits.max)
        )
    end

    return
end

function device_mixedinteger_rateofchange_outages!(
    optimization_container::PSI.OptimizationContainer,
    rate_data::Vector{DeviceOutageRampConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
    update_ref::PSI.UpdateRef,
)
    parameters = PSI.model_has_parameters(optimization_container)
    time_steps = PSI.model_time_steps(optimization_container)
    up_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "up")
    down_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "dn")

    variable = PSI.get_variable(optimization_container, var_names[1])
    varstart = PSI.get_variable(optimization_container, var_names[2])
    varstop = PSI.get_variable(optimization_container, var_names[3])

    set_name = [PSI.get_component_name(r) for r in rate_data]
    con_up = PSI.add_cons_container!(optimization_container, up_name, set_name, time_steps)
    con_down =
        PSI.add_cons_container!(optimization_container, down_name, set_name, time_steps)
    container_outage = PSI.get_parameter_container(optimization_container, update_ref)
    outage_parameter = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for r in rate_data
        name = PSI.get_component_name(r)
        ic_power = PSI.get_value(get_ic_power(r))
        ic_outage = PSI.get_value(get_ic_outage(r))
        @debug "add rate_of_change_constraint" name ic_power
        @assert (parameters && isa(ic_power, PJ.ParameterRef)) || !parameters
        expression_ub = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(optimization_container, val)[name, 1],
            )
        end
        con_up[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            expression_ub - (ic_power) <=
            r.ramp_limits.up + r.limits.max * varstart[name, 1]
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, 1] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(optimization_container, val)[name, 1],
                -1.0,
            )
        end
        con_down[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            (ic_power) - expression_lb <=
            r.ramp_limits.down + r.limits.max * varstop[name, 1]
        )
    end

    for t in time_steps[2:end], r in rate_data
        name = PSI.get_component_name(r)
        expression_ub = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_ub
            JuMP.add_to_expression!(
                expression_ub,
                PSI.get_variable(optimization_container, val)[name, t],
            )
        end
        con_up[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            expression_ub - variable[name, t - 1] <=
            r.ramp_limits.up + r.limits.max * varstart[name, t]
        )
        expression_lb = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
        for val in r.additional_terms_lb
            JuMP.add_to_expression!(
                expression_lb,
                PSI.get_variable(optimization_container, val)[name, t],
                -1.0,
            )
        end
        con_down[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            variable[name, t - 1] - expression_lb <=
            r.ramp_limits.down + r.limits.max * varstop[name, t]
        )
    end

    return
end

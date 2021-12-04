# TODO: docstrings
@doc raw"""

"""
function device_duration_look_ahead_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceDurationConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
)
    time_steps = PSI.model_time_steps(optimization_container)
    varon = PSI.get_variable(optimization_container, var_names[1])
    varstart = PSI.get_variable(optimization_container, var_names[2])
    varstop = PSI.get_variable(optimization_container, var_names[3])

    name_up = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "up")
    name_down = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "dn")

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_up = PSI.add_cons_container!(optimization_container, name_up, set_names, time_steps)
    con_down =
        PSI.add_cons_container!(optimization_container, name_down, set_names, time_steps)

    for t in time_steps
        for cont in constraint_info
            name = PSI.get_component_name(cont)
            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - cont.duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= cont.duration_data.up
                expr_up += first(cont.initial_duration).value
            end
            con_up[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] * cont.duration_data.up <=
                expr_up + (1 - cont.timeseries[t]) * cont.duration_data.up
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - cont.duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= cont.duration_data.down
                expr_dn += last(cont.initial_duration).value
            end
            if t == 1
                expr_dn += (1 - cont.initial_outage.value) * cont.duration_data.down
            else
                expr_dn += (1 - cont.timeseries[t - 1]) * cont.duration_data.down
            end
            con_down[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] * cont.duration_data.down <=
                expr_dn + (1 - cont.timeseries[t]) * cont.duration_data.down
            )
        end
    end
    return
end

# TODO: docstrings
@doc raw"""

"""
function device_duration_parameters_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceDurationConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
    param_reference::PSI.UpdateRef,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varon = PSI.get_variable(optimization_container, var_names[1])
    varstart = PSI.get_variable(optimization_container, var_names[2])
    varstop = PSI.get_variable(optimization_container, var_names[3])

    name_up = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "up")
    name_down = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "dn")

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_up = PSI.add_cons_container!(optimization_container, name_up, set_names, time_steps)
    con_down =
        PSI.add_cons_container!(optimization_container, name_down, set_names, time_steps)

    container_outage = PSI.get_parameter_container(optimization_container, param_reference)
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for t in time_steps
        for cont in constraint_info
            name = PSI.get_component_name(cont)

            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - cont.duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= cont.duration_data.up
                expr_up += cont.initial_duration[1].value
            end

            con_up[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] * cont.duration_data.up <=
                expr_up +
                (1 * cont.duration_data.up - param[name, t] * cont.duration_data.up)
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - cont.duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= cont.duration_data.down
                expr_dn += cont.initial_duration[2].value
            end

            if t == 1
                expr_dn += (
                    cont.duration_data.down -
                    cont.initial_outage.value * cont.duration_data.down
                )
            else
                expr_dn +=
                    (cont.duration_data.down - param[name, t - 1] * cont.duration_data.down)
            end
            con_down[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] * cont.duration_data.down <= expr_dn
            )
        end
    end
    return
end

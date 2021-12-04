struct SemiContinuousOutagesFF <: PSI.AbstractAffectFeedForward
    binary_source_problem::Symbol
    affected_variables::Vector{Symbol}
    cache::Union{Nothing, Type{<:PSI.AbstractCache}}
    function SemiContinuousOutagesFF(
        binary_source_problem::AbstractString,
        affected_variables::Vector{<:AbstractString},
        cache::Union{Nothing, Type{<:PSI.AbstractCache}},
    )
        new(Symbol(binary_source_problem), Symbol.(affected_variables), cache)
    end
end

function SemiContinuousOutagesFF(; binary_source_problem, affected_variables)
    return SemiContinuousOutagesFF(binary_source_problem, affected_variables, nothing)
end

PSI.get_binary_source_problem(p::SemiContinuousOutagesFF) = p.binary_source_problem

function semicontinuousrange_outages_ff(
    optimization_container::PSI.OptimizationContainer,
    cons_name::Symbol,
    constraint_infos::Vector{PSI.DeviceRangeConstraintInfo},
    commitment_reference::PSI.UpdateRef,
    outage_reference::PSI.UpdateRef{T},
    var_name::Symbol,
) where {T <: PSY.StaticInjection}
    time_steps = PSI.model_time_steps(optimization_container)
    ub_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "ub")
    lb_name = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "lb")
    variable = PSI.get_variable(optimization_container, var_name)
    varon =
        PSI.get_variable(optimization_container, PSI.make_variable_name(AUXILIARY_ON, T))
    # Used to make sure the names are consistent between the variable and the infos
    axes = JuMP.axes(variable)
    set_names = [PSI.get_component_name(ci) for ci in constraint_infos]
    @assert axes[2] == time_steps
    commitment_container =
        PSI.add_param_container!(optimization_container, commitment_reference, set_names)
    commitment_multiplier = PSI.get_multiplier_array(commitment_container)
    commitment_param = PSI.get_parameter_array(commitment_container)

    outage_container = PSI.get_parameter_container(optimization_container, outage_reference)
    outage_param = PSI.get_parameter_array(outage_container)

    con_ub = PSI.add_cons_container!(optimization_container, ub_name, set_names, time_steps)
    con_lb = PSI.add_cons_container!(optimization_container, lb_name, set_names, time_steps)

    cons_aux_lb = PSI.add_cons_container!(
        optimization_container,
        PSI.make_constraint_name(AUXILIARY_ON_RANGE_LB, T),
        set_names,
        time_steps,
    )
    cons_aux_ub = PSI.add_cons_container!(
        optimization_container,
        PSI.make_constraint_name(AUXILIARY_ON_RANGE_UB, T),
        set_names,
        time_steps,
    )
    cons_aux = PSI.add_cons_container!(
        optimization_container,
        PSI.make_constraint_name(AUXILIARY_ON_RANGE, T),
        set_names,
        time_steps,
    )

    for constraint_info in constraint_infos
        name = PSI.get_component_name(constraint_info)
        ub_value = JuMP.upper_bound(variable[name, 1])
        lb_value = JuMP.lower_bound(variable[name, 1])
        @debug "SemiContinuousOutagesFF" name ub_value lb_value
        # default set to 1.0, as this implementation doesn't use multiplier
        commitment_multiplier[name] = 1.0
        commitment_param[name] = PSI.add_parameter(optimization_container.JuMPmodel, 1.0)
        for t in time_steps
            expression_ub = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
            for val in constraint_info.additional_terms_ub
                JuMP.add_to_expression!(
                    expression_ub,
                    get_variable(optimization_container, val)[name, t],
                )
            end
            expression_lb = JuMP.AffExpr(0.0, variable[name, t] => 1.0)
            for val in constraint_info.additional_terms_lb
                JuMP.add_to_expression!(
                    expression_lb,
                    get_variable(optimization_container, val)[name, t],
                    -1.0,
                )
            end
            con_ub[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                expression_ub <= ub_value * varon[name, t]
            )
            con_lb[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                expression_lb >= lb_value * varon[name, t]
            )

            cons_aux_lb[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varon[name, t] <= commitment_param[name]
            )
            cons_aux_ub[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varon[name, t] <= outage_param[name, t]
            )
            cons_aux[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varon[name, t] >= outage_param[name, t] + commitment_param[name] - 1.0
            )
        end
    end

    # If the variable was a lower bound != 0, not removing the LB can cause infeasibilities
    for v in variable
        if JuMP.has_lower_bound(v)
            @debug "lb reset" v
            JuMP.set_lower_bound(v, 0.0)
        end
    end

    return
end

function PSI.feedforward!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ff_model::SemiContinuousOutagesFF,
) where {T <: PSY.StaticInjection, D <: PSI.AbstractDeviceFormulation}
    bin_var = PSI.make_variable_name(PSI.get_binary_source_problem(ff_model), T)
    parameter_ref = PSI.UpdateRef{JuMP.VariableRef}(bin_var)
    outage_ref = PSI.UpdateRef{T}(OUTAGE, "outage")
    constraint_infos = Vector{PSI.DeviceRangeConstraintInfo}(undef, length(devices))
    for (ix, d) in enumerate(devices)
        name = PSY.get_name(d)
        limits = PSY.get_active_power_limits(d)
        constraint_info = PSI.DeviceRangeConstraintInfo(name, limits)
        PSI.add_device_services!(constraint_info, d, model)
        constraint_infos[ix] = constraint_info
    end
    for prefix in PSI.get_affected_variables(ff_model)
        var_name = PSI.make_variable_name(prefix, T)
        semicontinuousrange_outages_ff(
            optimization_container,
            PSI.make_constraint_name(PSI.FEEDFORWARD_BIN, T),
            constraint_infos,
            parameter_ref,
            outage_ref,
            var_name,
        )
    end
end

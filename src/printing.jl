function Base.show(io::IO, sequence::PSI.SimulationSequence)
    stages = PSI._count_stages(sequence.execution_order)
    println(io, "Feed Forward Chronology")
    println(io, "-----------------------\n")
    to = []
    from = String("")
    for (k, v) in sequence.feedforward
        println(io, "$(k[1]): $(typeof(v)) -> $(k[3])\n")
        to = String.(v.affected_variables)
        if isa(v, PSI.SemiContinuousFF) || isa(v, SemiContinuousOutagesFF)
            from = String.(v.binary_source_problem)
        elseif isa(v, PSI.RangeFF)
            from = String.([v.variable_source_problem_ub, v.variable_source_problem_lb])
        else
            from = String.(v.variable_source_problem)
        end
        PSI._print_feedforward(io, sequence.feedforward_chronologies, to, from)
    end
    println(io, "Initial Condition Chronology")
    println(io, "----------------------------\n")
    if sequence.ini_cond_chronology == PSI.IntraProblemChronology()
        PSI._print_intra_stages(io, stages)
    elseif sequence.ini_cond_chronology == PSI.InterProblemChronology()
        PSI._print_inter_stages(io, stages)
    end
end

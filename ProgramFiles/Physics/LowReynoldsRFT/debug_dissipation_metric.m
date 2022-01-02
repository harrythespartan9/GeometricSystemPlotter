function Mp = debug_dissipation_metric(geometry,physics,shapeparams)

physics_function = @LowRE_metric_discrete;

Mp = physics_function(geometry,physics,shapeparams);

end
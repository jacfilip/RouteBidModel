using Pkg
using DataFrames
using Random
using Distributions
using CSV
using Plots
pyplot()

using Revise
Pkg.activate(".")
using RouteBidModel

Random.seed!(1);

#simulation of bidding behavior
N_MAX=200
p2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60) # km/h
)

neq = NashEq(p2)




costneq_pa=sum(cost(p2,neq))/p2.N_MAX
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 300, optimizebid)

CSV.write(raw".\results\log_stepbifurcate.csv", log_step) #seed = 1
CSV.write(raw".\results\log_agbifurcate.csv", log_ag)

bidm, log_ag_m,log_step_m = play_global_nash(p2, p2.ct, 60, optimizebid_middle)
CSV.write(raw".\results\log_step_m.csv", log_step_m) #seed = 1
CSV.write(raw".\results\log_ag_m.csv", log_ag_m)

Random.seed!(0);
p2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60) # km/h
)
neq = NashEq(p2)
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 500, optimizebid) #seed = 0
CSV.write(raw".\results\log_stepcalmseed0.csv", log_step)
CSV.write(raw".\results\log_agcalmseed0.csv", log_ag)



function doplot(log_step::DataFrame, neq::NashEq)
  log_step.costneq_pa = (log_step.costneq1.*log_step.n0.+log_step.costneq2.*log_step.n1)./(log_step.n0+log_step.n1)
  @assert minimum(log_step.costneq_pa) ≈ maximum(log_step.costneq_pa)
  plot(1:nrow(log_step),log_step.cost_real_avg./log_step.costneq_pa, label="Total travel costs %",
   linewidth=4,
   linecolor=:black,
   yformatter = yi -> "\$$(round(Int,100yi))\$ %",
   xlabel = "Simulation steps",
   ylabel = "Value relative to Nash Equilibrium"
   )
  plot!(1:nrow(log_step),(log_step.cost1.+log_step.pmnt1) ./log_step.costneq1, label=raw"% netto cost route $s_0$", linewidth=2,linecolor=:red)
  plot!(1:nrow(log_step),(log_step.cost2.+log_step.pmnt2)./log_step.costneq2, label=raw"% netto cost route $s_1$", linewidth=2,linecolor=:green)
  plot!(1:nrow(log_step),log_step.n0./neq.n0, label=raw"% of cars route $s_0$",linestyle=:dash,linecolor=:red)
  plot!(1:nrow(log_step),log_step.n1./neq.n1, label=raw"% of cars route $s_1$",linestyle=:dash,linecolor=:green)
  plot!(1:nrow(log_step),log_step.t_1./log_step.neq_t, label=raw"% travel time route $s_0$",linestyle=:dot,linecolor=:red)
  plot!(1:nrow(log_step),log_step.t_2./log_step.neq_t, label=raw"% travel time route $s_1$",linestyle=:dot,linecolor=:green)

end

log_step = CSV.read(raw".\results\log_stepbifurcate.csv")
doplot(log_step[1:210,:], neq)
savefig("bifurcate.pdf")#seed=1
savefig("bifurcate.png")


log_step_m = CSV.read(raw".\results\log_step_m.csv")
doplot(log_step_m[1:47,:], neq)  # seed =1
savefig("degenerate.pdf")
savefig("degenerate.png")

log_step_calm = CSV.read(raw".\results\log_stepcalmseed0.csv")
doplot(log_step_calm[1:210,:],neq) #seed 0
savefig("calm_dyn.pdf")
savefig("calm_dyn.png")

using Pkg
using DataFrames
using Random
using Distributions
using CSV
using LaTeXStrings
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
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq = NashEq(p2)




costneq_pa=sum(cost(p2,neq))/p2.N_MAX
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 300, optimizebid)

CSV.write(raw".\results\log_stepbifurcate.csv", log_step) #seed = 1
CSV.write(raw".\results\log_agbifurcate.csv", log_ag)


Random.seed!(0);
p2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

bidm, log_ag_m,log_step_m = play_global_nash(p2, p2.ct, 100, optimizebid_middle)
CSV.write(raw".\results\log_step_m.csv", log_step_m) #seed = 0
CSV.write(raw".\results\log_ag_m.csv", log_ag_m)


neq = NashEq(p2)
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 220, optimizebid) #seed = 0
CSV.write(raw".\results\log_stepcalmseed0.csv", log_step)
CSV.write(raw".\results\log_agcalmseed0.csv", log_ag)



function doplot(log_step::DataFrame, neq::NashEq; kw...)
  log_step.costneq_pa = (log_step.costneq1.*log_step.n0.+log_step.costneq2.*log_step.n1)./(log_step.n0+log_step.n1)
  @assert minimum(log_step.costneq_pa) ≈ maximum(log_step.costneq_pa)
  plot(1:nrow(log_step),log_step.cost_real_avg./log_step.costneq_pa, label="Total costs: "*L"\sum C^*_k / \sum C^\dagger_k",
   linewidth=4,
   size=(720,460),
   linecolor=:black,
   yformatter = yi -> "\$$(round(Int,100yi))\$ %",
   xlabel = "Simulation steps",
   ylabel = "Value relative to Nash Equilibrium";
   kw...
   )
  plot!(1:nrow(log_step),(log_step.cost1.+log_step.pmnt1) ./log_step.costneq1, label=raw"Costs route $s_0$: "*L"((C^*)^T(1-\mathbf{x}^*)) /((C^\dagger)^T(1-\mathbf{x}^*))", linewidth=2,linecolor=:red)
  plot!(1:nrow(log_step),(log_step.cost2.+log_step.pmnt2)./log_step.costneq2, label=raw"Costs route $s_1$: "*L"((C^*)^T\mathbf{x}^*) /((C^\dagger)^T\mathbf{x}^*)", linewidth=2,linecolor=:green)
  plot!(1:nrow(log_step),log_step.n0./neq.n0, label=raw"Cars on route $s_0$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:red)
  plot!(1:nrow(log_step),log_step.n1./neq.n1, label=raw"Cars on route $s_1$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:green)
  plot!(1:nrow(log_step),log_step.t_1./log_step.neq_t, label=raw"Travel time route $s_0$: "*L"t_0^*/t_0^\dagger",linestyle=:dot,linecolor=:red)
  plot!(1:nrow(log_step),log_step.t_2./log_step.neq_t, label=raw"Travel time route $s_1$: "*L"t_1^*/t_1^\dagger",linestyle=:dot,linecolor=:green)

end

log_step = CSV.read(raw".\results\log_stepbifurcate.csv")
doplot(log_step[1:220,:], neq)
savefig("bifurcate.png")
savefig("bifurcate.pdf")#seed=1


log_step_m = CSV.read(raw".\results\log_step_m.csv")
doplot(log_step_m[1:100,:], neq; ylim=(-1.7,3.3))  # seed =0
#ylim=(-1.1,8.1),yticks=-1:7
savefig("degenerate.pdf")
savefig("degenerate.png")

log_step_calm = CSV.read(raw".\results\log_stepcalmseed0.csv")
doplot(log_step_calm[1:210,:],neq;ylim=(0.6,1.6)) #seed 0
savefig("calm_dyn.pdf")
savefig("calm_dyn.png")

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

solve_middle_payment(p2)

bidm, log_ag_m,log_step_m = play_global_nash(p2, p2.ct, 220, optimizebid_middle)
CSV.write(raw".\results\log_step_m.csv", log_step_m) #seed = 0
CSV.write(raw".\results\log_ag_m.csv", log_ag_m)


neq = NashEq(p2)
bid, log_ag, log_step = play_global_nash(p2, p2.ct, 220, optimizebid) #seed = 0
CSV.write(raw".\results\log_stepcalmseed0.csv", log_step)
CSV.write(raw".\results\log_agcalmseed0.csv", log_ag)



function doplot(log_step::DataFrame, neq::NashEq; kw...)
  log_step.costneq_pa = (log_step.costneq1.*log_step.n0.+log_step.costneq2.*log_step.n1)./(log_step.n0+log_step.n1)
  @assert minimum(log_step.costneq_pa) ≈ maximum(log_step.costneq_pa)
  plot(1:nrow(log_step),log_step.cost_real_avg./log_step.costneq_pa, label="Total costs: "*L"C^*_{TOT} / C^\dagger_{TOT}",
   linewidth=4,
   size=(720,460),
   linecolor=:black,
   yformatter = yi -> "\$$(round(Int,100yi))\$ %",
   xlabel = "Simulation steps",
   ylabel = "Value relative to Nash Equilibrium";
   kw...
   )
  plot!(1:nrow(log_step),(log_step.cost1.+log_step.pmnt1) ./log_step.costneq1, label=raw"Net costs route $s_0$: "*L"((\mathbf{C}^*-\mathbf{p})^T(\mathbf{1}-\mathbf{x}^*)) /((\mathbf{C}^\dagger)^T(\mathbf{1}-\mathbf{x}^*))", linewidth=2,linecolor=:red)
  plot!(1:nrow(log_step),(log_step.cost2.+log_step.pmnt2)./log_step.costneq2, label=raw"Net costs route $s_1$: "*L"((\mathbf{C}^*-\mathbf{p})^T\mathbf{x}^*) /((\mathbf{C}^\dagger)^T\mathbf{x}^*)", linewidth=2,linecolor=:green)
  plot!(1:nrow(log_step),log_step.n0./neq.n0, label=raw"Cars on route $s_0$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:red)
  plot!(1:nrow(log_step),log_step.n1./neq.n1, label=raw"Cars on route $s_1$: "*L"n_0^*/n_0^\dagger",linestyle=:dash,linecolor=:green)
  plot!(1:nrow(log_step),log_step.t_1./log_step.neq_t, label=raw"Travel time route $s_0$: "*L"t_0^*/t^\dagger",linestyle=:dot,linecolor=:red)
  plot!(1:nrow(log_step),log_step.t_2./log_step.neq_t, label=raw"Travel time route $s_1$: "*L"t_1^*/t^\dagger",linestyle=:dot,linecolor=:green)

end

log_step = CSV.read(raw".\results\log_stepbifurcate.csv")
doplot(log_step[1:220,:], neq)
savefig("bifurcate.png")
savefig("bifurcate.pdf")#seed=1


log_step_m = CSV.read(raw".\results\log_step_m.csv")
doplot(log_step_m[1:220,:], neq; ) #ylim=(-1.7,3.3)  # seed =0
#ylim=(-1.1,8.1),yticks=-1:7
savefig("degenerate.pdf")
savefig("degenerate.png")

log_step_calm = CSV.read(raw".\results\log_stepcalmseed0.csv")
doplot(log_step_calm[1:210,:],neq;ylim=(0.6,1.6)) #seed 0
savefig("calm_dyn.pdf")
savefig("calm_dyn.png")

#heterogenity sensitivity
###################################
#scenario 1 LN(3,1.5)
Random.seed!(0);
p2_1 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,1.5),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq_1 = NashEq(p2_1)
#solve_middle_payment(p2_1)

bidm_1, log_ag_1,log_step_1 = play_global_nash(p2_1, p2_1.ct, 300, optimizebid)
#CSV.write(raw".\results\log_step_1.csv", log_step_1) #seed = 0
#CSV.write(raw".\results\log_ag_1.csv", log_ag_1)
#log_step_1 = CSV.read(raw".\results\log_step_1.csv")
doplot(log_step_1[1:220,:], neq_1)
savefig("sensitivity_3_15.png")

#######################################
#scenario 2 LN(3,2)
Random.seed!(0);
p2_2 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,2),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq_2 = NashEq(p2_2)

bidm_2, log_ag_2,log_step_2 = play_global_nash(p2_2, p2_2.ct, 300, optimizebid)
doplot(log_step_2[1:220,:], neq_2)
savefig("sensitivity_3_2.png")

#######################################
#scenario 3 LN(3,0.75)
Random.seed!(0);
p2_3 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,0.75),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq_3 = NashEq(p2_3)

bidm_3, log_ag_3,log_step_3 = play_global_nash(p2_3, p2_3.ct, 300, optimizebid)
doplot(log_step_3[1:220,:], neq_3)
savefig("sensitivity_3_075.png")

#######################################
#scenario 4 LN(3,0.5)
Random.seed!(0);
p2_4 = BidModelParams(N_MAX=N_MAX, N=(150,100),
  ct=rand(LogNormal(3,0.5),N_MAX), # $/h
  d=(10.0, 10.0), # km
  v_max=(60, 60), # km/h
  v_min=(5, 5), # km/h
  cf=0
)

neq_4 = NashEq(p2_4)

bidm_4, log_ag_4,log_step_4 = play_global_nash(p2_4, p2_4.ct, 300, optimizebid)
doplot(log_step_4[1:220,:], neq_4)
savefig("sensitivity_LN_3_05.png")

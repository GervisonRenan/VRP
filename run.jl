include("instanceVRP.jl")
include("modelVRP.jl")

export open_archive, build_vrp_model, solve_vrp, show_results, create_test_instance

file = "C:/Users/budun/OneDrive/Área de Trabalho/----/Estudos/GEEOC/PIBITI/Códigos estudos/Projeto 3/A/A-n32-k5.vrp"
coords, instance = open_archive(file)
result = solve_vrp(instance)
show_results = (instance, result)

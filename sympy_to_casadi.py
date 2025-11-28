import sympy as sp
import casadi as cas


def get_casadi_variables(var: list[sp.symbols]):
    casadi_variables = []
    for v in var:
        # All sympy symbols are of shape 1x1
        casadi_variables += [cas.MX.sym(str(v), 1, 1)]
    return casadi_variables

# def sympy_to_casadi(expr: sp.Function, var: list[sp.symbols]):
#     symbol_map = {}
#     dynamic_symbol_map = {}
#
#     def get_casadi_symbol(sym):
#         if sym not in symbol_map:
#             symbol_map[sym] = cas.MX.sym(str(sym))
#         return symbol_map[sym]
#
#     def get_casadi_dynamic_symbol(func):
#         name = str(func.func)
#         if name not in dynamic_symbol_map:
#             dynamic_symbol_map[name] = cas.MX.sym(name)
#         return dynamic_symbol_map[name]
#
#     def convert(expr: sp.matrices.dense.MutableDenseMatrix):
#         # Case 1: Symbol
#         if isinstance(expr, sp.Symbol):
#             return get_casadi_symbol(expr)
#
#         # Case 2: Number
#         if hasattr(expr, 'is_Number') and expr.is_Number:
#             return cas.MX(float(expr))
#
#         # Case 3: Function — handle math functions first, then dynamic symbols
#         if isinstance(expr, sp.Function):
#             func_name = expr.func.__name__
#             casadi_func_map = {
#                 'sin': cas.sin,
#                 'cos': cas.cos,
#                 'tan': cas.tan,
#                 'asin': cas.asin,
#                 'acos': cas.acos,
#                 'atan': cas.atan,
#                 'sinh': cas.sinh,
#                 'cosh': cas.cosh,
#                 'tanh': cas.tanh,
#                 'asinh': cas.asinh,
#                 'acosh': cas.acosh,
#                 'atanh': cas.atanh,
#                 'exp': cas.exp,
#                 'log': cas.log,
#                 'sqrt': cas.sqrt,
#                 'Abs': cas.fabs,
#                 'sign': cas.sign,
#                 'floor': cas.floor,
#                 'ceiling': cas.ceil,
#                 'erf': cas.erf,
#                 # 'erfc': cas.erfc,
#             }
#             if func_name in casadi_func_map:
#                 return casadi_func_map[func_name](convert(expr.args[0]))
#             else:
#                 return get_casadi_dynamic_symbol(expr)
#
#         # Case 4: Addition
#         if isinstance(expr, sp.Add):
#             return sum(convert(arg) for arg in expr.args)
#
#         # Case 5: Multiplication
#         if isinstance(expr, sp.Mul):
#             result = 1
#             for arg in expr.args:
#                 result *= convert(arg)
#             return result
#
#         # Case 6: Power
#         if isinstance(expr, sp.Pow):
#             base, exp = expr.args
#             return convert(base) ** convert(exp)
#
#         # Case 7: Matrix
#         if isinstance(expr, sp.MatrixBase):
#             rows, cols = expr.shape
#             cas_matrix = cas.MX.zeros(rows, cols)
#             for i in range(rows):
#                 for j in range(cols):
#                     cas_matrix[i, j] = convert(expr[i, j])
#             return cas_matrix
#
#         raise NotImplementedError(f"Unsupported SymPy expression type: {type(expr)}")
#
#     output_casadi = convert(expr)
#     casadi_variables = get_casadi_variables(var)
#     return output_casadi, casadi_variables


def sympy_to_casadi_2(function_name: str, expr: sp.Function, var: list[sp.symbols]):
    from sympy.utilities.lambdify import lambdify

    casadi_variables = get_casadi_variables(var)
    # x
    bike_v1_0_q1 = var[0][0]
    bike_v1_0_q2 = var[0][1]
    bike_v1_0_q3 = var[0][2]
    bike_v1_0_q4 = var[0][3]
    bike_v1_0_q6 = var[0][4]
    bike_v1_0_q7 = var[0][5]
    bike_v1_0_q8 = var[0][6]
    bike_v1_0_q5 = var[0][7]
    bike_v1_0_u6 = var[0][8]
    bike_v1_0_u7 = var[0][9]
    bike_v1_0_u1 = var[0][10]
    bike_v1_0_u2 = var[0][11]
    bike_v1_0_u3 = var[0][12]
    bike_v1_0_u5 = var[0][13]
    bike_v1_0_u8 = var[0][14]
    # steer torque
    steer_torque = var[1][0]
    # disturbance
    disturbance = var[2][0]

    casadi_mapping = {'ImmutableDenseMatrix': cas.blockcat,
               'MutableDenseMatrix': cas.blockcat,
               'Abs': cas.fabs,
               'sin': cas.sin,
               'cos': cas.cos,
               'tan': cas.tan,
               'asin': cas.asin,
               'acos': cas.acos,
               'atan': cas.atan,
               'sinh': cas.sinh,
               'cosh': cas.cosh,
               'tanh': cas.tanh,
               'asinh': cas.asinh,
               'acosh': cas.acosh,
               'atanh': cas.atanh,
               'exp': cas.exp,
               'log': cas.log,
               'sqrt': cas.sqrt,
               'bike_v1_0_q1(t)': bike_v1_0_q1,
                'bike_v1_0_q2(t)': bike_v1_0_q2,
                'bike_v1_0_q3(t)': bike_v1_0_q3,
                'bike_v1_0_q4(t)': bike_v1_0_q4,
                'bike_v1_0_q6(t)': bike_v1_0_q6,
                'bike_v1_0_q7(t)': bike_v1_0_q7,
                'bike_v1_0_q8(t)': bike_v1_0_q8,
                'bike_v1_0_q5(t)': bike_v1_0_q5,
                'bike_v1_0_u6(t)': bike_v1_0_u6,
                'bike_v1_0_u7(t)': bike_v1_0_u7,
                'bike_v1_0_u1(t)': bike_v1_0_u1,
                'bike_v1_0_u2(t)': bike_v1_0_u2,
                'bike_v1_0_u3(t)': bike_v1_0_u3,
                'bike_v1_0_u5(t)': bike_v1_0_u5,
                'bike_v1_0_u8(t)': bike_v1_0_u8,
                'steer_torque': steer_torque,
                'disturbance': disturbance,
               }
    f = lambdify(var, expr, modules=casadi_mapping)

    casadi_output = f(*casadi_variables)
    casadi_func = cas.Function(function_name, casadi_variables, [f(*casadi_variables)])

    return casadi_func

# --- Exemple de test ---
if __name__ == "__main__":

    # Définir des symboles SymPy
    x, y = sp.symbols('x y')
    t = sp.symbols('t')
    # The following line is commented out because CasADi does not handle dynamic functions
    # bike_q7 = sp.Function('bike_v1_0_q7')(t)  # Fonction dynamique

    # Créer une matrice SymPy 2x2
    M = sp.Matrix([
        [x**2 + sp.sin(y), sp.cos(x) * y],
        [sp.exp(x), sp.Abs(y - 1)]
    ])

    # Convertir en CasADi
    casadi_func = sympy_to_casadi_2("M", expr=M, var=[x, y, t])
    output_casadi = casadi_func(0.1, 0.2, 0.3)

    # Afficher le résultat
    print("Matrice CasADi :")
    print(output_casadi)

    # Vérifier les types
    print("\nType de chaque élément :")
    for i in range(output_casadi.size1()):
        for j in range(output_casadi.size2()):
            print(f"M[{i},{j}] = {output_casadi[i,j]} (type: {type(output_casadi[i,j])})")

    print("\nTest terminé avec succès.")
import sympy as sp
import casadi as ca

def sympy_to_casadi(expr, symbol_map=None, dynamic_symbol_map=None):
    if symbol_map is None:
        symbol_map = {}
    if dynamic_symbol_map is None:
        dynamic_symbol_map = {}

    def get_casadi_symbol(sym):
        if sym not in symbol_map:
            symbol_map[sym] = ca.SX.sym(str(sym))
        return symbol_map[sym]

    def get_casadi_dynamic_symbol(func):
        name = str(func.func)
        if name not in dynamic_symbol_map:
            dynamic_symbol_map[name] = ca.SX.sym(name)
        return dynamic_symbol_map[name]

    def convert(expr):
        # Case 1: Symbol
        if isinstance(expr, sp.Symbol):
            return get_casadi_symbol(expr)

        # Case 2: Number
        if hasattr(expr, 'is_Number') and expr.is_Number:
            return float(expr)

        # Case 3: Function — handle math functions first, then dynamic symbols
        if isinstance(expr, sp.Function):
            func_name = expr.func.__name__
            casadi_func_map = {
                'sin': ca.sin,
                'cos': ca.cos,
                'tan': ca.tan,
                'asin': ca.asin,
                'acos': ca.acos,
                'atan': ca.atan,
                'sinh': ca.sinh,
                'cosh': ca.cosh,
                'tanh': ca.tanh,
                'asinh': ca.asinh,
                'acosh': ca.acosh,
                'atanh': ca.atanh,
                'exp': ca.exp,
                'log': ca.log,
                'sqrt': ca.sqrt,
                'Abs': ca.fabs,
                'sign': ca.sign,
                'floor': ca.floor,
                'ceiling': ca.ceil,
                'erf': ca.erf,
                # 'erfc': ca.erfc,
            }
            if func_name in casadi_func_map:
                return casadi_func_map[func_name](convert(expr.args[0]))
            else:
                return get_casadi_dynamic_symbol(expr)

        # Case 4: Addition
        if isinstance(expr, sp.Add):
            return sum(convert(arg) for arg in expr.args)

        # Case 5: Multiplication
        if isinstance(expr, sp.Mul):
            result = 1
            for arg in expr.args:
                result *= convert(arg)
            return result

        # Case 6: Power
        if isinstance(expr, sp.Pow):
            base, exp = expr.args
            return convert(base) ** convert(exp)

        # Case 7: Matrix
        if isinstance(expr, sp.MatrixBase):
            rows, cols = expr.shape
            cas_matrix = ca.SX.zeros(rows, cols)
            for i in range(rows):
                for j in range(cols):
                    cas_matrix[i, j] = convert(expr[i, j])
            return cas_matrix

        raise NotImplementedError(f"Unsupported SymPy expression type: {type(expr)}")

    return(convert(expr), dynamic_symbol_map)

# --- Exemple de test ---
if __name__ == "__main__":
    # Définir des symboles SymPy
    x, y = sp.symbols('x y')
    t = sp.symbols('t')
    bike_q7 = sp.Function('bike_v1_0_q7')(t)  # Fonction dynamique

    # Créer une matrice SymPy 2x2
    M = sp.Matrix([
        [x**2 + sp.sin(y), bike_q7],
        [sp.exp(x), sp.Abs(y - 1)]
    ])

    # Convertir en CasADi
    M_casadi = sympy_to_casadi(M)

    # Afficher le résultat
    print("Matrice CasADi :")
    print(M_casadi)

    # Vérifier les types
    print("\nType de chaque élément :")
    for i in range(M_casadi.size1()):
        for j in range(M_casadi.size2()):
            print(f"M[{i},{j}] = {M_casadi[i,j]} (type: {type(M_casadi[i,j])})")
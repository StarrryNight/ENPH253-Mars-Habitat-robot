Import("env")
env.Tool("compilation_db")
env.CompilationDatabase("$PROJECT_DIR/compile_commands.json")

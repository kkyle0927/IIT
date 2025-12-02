@echo off
for /r .\Control_Module\AGR_Library_CM\ %%f in (*.c) do (
    clang-format -i -style=Microsoft "%%f"
)

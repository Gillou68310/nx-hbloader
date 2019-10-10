#ifndef __GDBSTUB_H
#define __GDBSTUB_H

#ifdef __cplusplus
extern "C" {
#endif

    int GDBStub_init();
    void GDBStub_shutdown();
	void GDBStub_add_execute_breakpoint(u64 addr);

#ifdef __cplusplus
} //end extern "C"
#endif

#endif /* __GDBSTUB_H */
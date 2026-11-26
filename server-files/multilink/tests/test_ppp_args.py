import os, sys, types
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.modules.setdefault('telnet_door', types.ModuleType('telnet_door'))
import server


def test_the_first_pppd_waits_for_the_caller():
    assert 'silent' in server.ppp_arguments()


def test_a_fresh_pppd_after_a_failed_reattach_opens_lcp_itself():
    # 2026-11-16: the caller was already in IPCP; a silent pppd waited for
    # an LCP frame that never came, and the caller gave up after 30 s.
    assert 'silent' not in server.ppp_arguments(silent=False)

use bcm2835_lpa::PM;

pub fn restart(pm: &PM) -> ! {
    // get 12 bits for wdog(), <time> = .time * 16 at <clock>
    const WDOG_TIME: u32 = 0x00f;

    pm.wdog()
        .write(|w| w.passwd().passwd().time().variant(WDOG_TIME));
    pm.rstc()
        .modify(|_, w| w.passwd().passwd().wrcfg().full_reset());

    loop {}
}

use anyhow::Result;

pub struct Fsm<C> {
    state: Box<dyn State<C>>,
    ctx: C,
}

pub trait State<C> {
    fn advance(&mut self) -> Result<Box<dyn State<C>>>;
}

impl<C> Fsm<C> {
    pub fn advance(&mut self) -> Result<()> {
        let state = self.state.take().expect("FSM has no state");
        self.state = Some(state.advance(&mut self.ctx)?);
        Ok(())
    }
}

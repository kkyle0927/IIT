# Top 5 Model Cards

## P6: switch_dualhead
- Type: `AssistSwitchHeadTCN_GRU`
- Main idea: shared backbone, but final prediction head switches between assist-off and assist-on behavior
- Intended benefit: keep common gait representation while separating assist-domain calibration
- Practical read: strongest overall result in this archive

## P4: onbias_corrector
- Type: `AssistOnBiasCorrectorTCN_GRU`
- Main idea: nominal predictor stays shared, but assist-on windows get a stronger slow bias corrector
- Intended benefit: specifically reduce high-assist steady offset without rebuilding the whole model
- Practical read: likely strongest simple deployable fix

## P7: regime_mixture
- Type: `RegimeExpertTCN_GRU`
- Main idea: mixture-style regime experts for different assist contexts
- Intended benefit: handle regime-dependent mapping more explicitly
- Practical read: competitive, but less clean than P6/P4

## R1: d1_reference
- Type: `PersistentBiasTCN_GRU`
- Main idea: persistent slow bias state carried through the sequence
- Intended benefit: reduce plateau drift through a slow streaming-like calibration state
- Practical read: strong RMSE reference, but slower than the other top candidates

## P8: split_off_on
- Type: `AssistSplitTCN_GRU`
- Main idea: full assist-off and assist-on split model
- Intended benefit: remove assist domain shift by direct model separation
- Practical read: did not beat shared-backbone routed variants


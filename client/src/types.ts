
export interface LoggingPayload {
  idx: number
  f_idx: number
  pos: number
  p_temp: number
  h_dur: number
  t: number
  p: number
  h: number
  gas: number
  lbl: number
}

export interface InferencePayload {
  prediction: string
  scores: Record<string, number>
}

export interface Message<T> {
  op: number
  d: T
}

export enum Mode {
  Logging = 1,
  Inference = 2
}
